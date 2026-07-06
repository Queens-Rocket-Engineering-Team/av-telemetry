#include <esp_err.h>
#include <esp_log.h>
#include <fcntl.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>

#include "qlcp_lib.h"
#include "wifi_tools.h"

// RX assembler: packets are framed by the 9-byte QLCP header whose
// packet_length field is the TOTAL length (header included).
#define RX_ASM_LEN 2048
// TX pending buffer: must hold the largest packet (CONFIG json ~2.4 KB).
#define TX_PENDING_LEN 4096

// Per-tick budgets — keep every service call bounded under the 2 s task WDT.
#define RX_BYTES_PER_TICK 512
#define TX_BYTES_PER_TICK 1024

static const char *TAG = "TCP";

static uint8_t s_rxBuf[RX_ASM_LEN];
static uint16_t s_rxHave = 0;
static uint16_t s_rxNeed = QLCP_HEADER_SIZE;

static uint8_t s_txBuf[TX_PENDING_LEN];
static size_t s_txLen = 0;
static size_t s_txOff = 0;

static void tcp_reset_streams(void) {
    s_rxHave = 0;
    s_rxNeed = QLCP_HEADER_SIZE;
    s_txLen = 0;
    s_txOff = 0;
}

esp_err_t tcp_connect_begin(net_link_t *link) {
    if (link == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char server_port_str[10] = {0};
    snprintf(server_port_str, sizeof(server_port_str), "%u", link->server_tcp_port);

    esp_err_t ret = ESP_FAIL;
    int32_t sock = -1;
    link->tcp_sock = -1;

    int32_t err;
    struct addrinfo hints = {0}, *res = NULL;
    // set up TCP parameters for socket (numeric IP — no DNS lookup happens)
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    err = getaddrinfo(link->server_ip, server_port_str, &hints, &res);
    if (err != 0) {
        goto cleanup;
    }

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        goto cleanup;
    }

    int32_t enable = 1;
    err = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    if (err != 0) {
        goto cleanup;
    }

    // non-blocking BEFORE connect: EINPROGRESS is the expected pending state,
    // completion is polled by tcp_connect_service()
    err = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    if (err != 0) {
        goto cleanup;
    }

    err = connect(sock, res->ai_addr, res->ai_addrlen);
    if (err != 0 && errno != EINPROGRESS) {
        ESP_LOGE(TAG, "connect failed: errno %d", errno);
        goto cleanup;
    }

    tcp_reset_streams();
    link->tcp_sock = sock;
    ret = ESP_OK;

cleanup:
    if (res != NULL) {
        freeaddrinfo(res);
    }
    if (sock != -1 && ret != ESP_OK) {
        close(sock);
    }
    return ret;
}

int tcp_connect_service(net_link_t *link) {
    if (link == NULL || link->tcp_sock < 0) {
        return -1;
    }

    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(link->tcp_sock, &wfds);
    struct timeval tv = {0, 0}; // zero-timeout poll

    int err = select(link->tcp_sock + 1, NULL, &wfds, NULL, &tv);
    if (err < 0) {
        ESP_LOGE(TAG, "select failed: errno %d", errno);
        return -1;
    }
    if (err == 0 || !FD_ISSET(link->tcp_sock, &wfds)) {
        return 0; // still connecting
    }

    int sockErr = 0;
    socklen_t len = sizeof(sockErr);
    if (getsockopt(link->tcp_sock, SOL_SOCKET, SO_ERROR, &sockErr, &len) != 0 || sockErr != 0) {
        ESP_LOGE(TAG, "connect failed: SO_ERROR %d", sockErr);
        return -1;
    }

    ESP_LOGI(TAG, "Connected to server");
    return 1;
}

int tcp_rx_service(net_link_t *link, qlcp_client_payload *out) {
    if (link == NULL || out == NULL || link->tcp_sock < 0) {
        return -1;
    }

    size_t budget = RX_BYTES_PER_TICK;

    // at most two recv calls per tick: finish the header, then the body
    for (int pass = 0; pass < 2; pass++) {
        if (s_rxHave < s_rxNeed) {
            size_t want = (size_t)(s_rxNeed - s_rxHave);
            if (want > budget) {
                want = budget;
            }
            ssize_t n = recv(link->tcp_sock, s_rxBuf + s_rxHave, want, MSG_DONTWAIT);
            if (n == 0) {
                ESP_LOGI(TAG, "Connection closed by server gracefully");
                return -1;
            }
            if (n < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    return 0;
                }
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                return -1;
            }
            s_rxHave += (uint16_t)n;
            budget -= (size_t)n;
        }
        if (s_rxHave < s_rxNeed) {
            return 0;
        }

        if (s_rxNeed == QLCP_HEADER_SIZE) {
            uint16_t totalLen = 0;
            if (qlcp_get_packet_len(&totalLen, s_rxBuf, s_rxHave) != QLCP_OK ||
                totalLen < QLCP_HEADER_SIZE || totalLen > sizeof(s_rxBuf)) {
                ESP_LOGE(TAG, "bad packet length %u — resync via reconnect", totalLen);
                return -1;
            }
            s_rxNeed = totalLen;
            if (s_rxHave < s_rxNeed) {
                continue; // read the body within this tick's budget
            }
        }

        // full packet assembled
        qlcp_lib_ret ret = qlcp_decode_server_to_client(out, s_rxBuf, s_rxHave);
        s_rxHave = 0;
        s_rxNeed = QLCP_HEADER_SIZE;
        if (ret != QLCP_OK) {
            ESP_LOGE(TAG, "QRET protocol err: %d", ret);
            return 0; // framing intact — skip the bad packet
        }
        return 1;
    }
    return 0;
}

int tcp_tx_service(net_link_t *link) {
    if (link == NULL || link->tcp_sock < 0) {
        return -1;
    }
    if (s_txLen == 0) {
        return 1; // idle
    }

    size_t want = s_txLen - s_txOff;
    if (want > TX_BYTES_PER_TICK) {
        want = TX_BYTES_PER_TICK;
    }
    ssize_t n = send(link->tcp_sock, s_txBuf + s_txOff, want, MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return 0;
        }
        ESP_LOGE(TAG, "send failed: errno %d", errno);
        return -1;
    }
    s_txOff += (size_t)n;
    if (s_txOff >= s_txLen) {
        s_txLen = 0;
        s_txOff = 0;
        return 1;
    }
    return 0;
}

int tcp_tx_payload(net_link_t *link, const qlcp_server_payload *payload) {
    if (link == NULL || payload == NULL || link->tcp_sock < 0) {
        return -1;
    }
    if (s_txLen != 0) {
        return 1; // previous packet still draining — caller retries next tick
    }

    size_t packet_len = sizeof(s_txBuf);
    qlcp_lib_ret ret;

    // encode immediately: payloads carrying pointers (status/data) reference
    // caller-stack arrays that are only valid during this call
    switch (payload->packet_type) {
    case QLCP_PT_CONFIG:
        ret = qlcp_encode_config(s_txBuf, &packet_len, &payload->payload_data.config);
        break;
    case QLCP_PT_DATA:
        ret = qlcp_encode_data(s_txBuf, &packet_len, &payload->payload_data.data);
        break;
    case QLCP_PT_STATUS:
        ret = qlcp_encode_status(s_txBuf, &packet_len, &payload->payload_data.status);
        break;
    case QLCP_PT_ACK:
        ret = qlcp_encode_ack(s_txBuf, &packet_len, &payload->payload_data.ack);
        break;
    case QLCP_PT_NACK:
        ret = qlcp_encode_nack(s_txBuf, &packet_len, &payload->payload_data.nack);
        break;
    default:
        ESP_LOGE(TAG, "Attempted to send invalid server packet");
        return -1;
    }
    if (ret != QLCP_OK) {
        ESP_LOGE(TAG, "QLCP err: %d", ret);
        return -1;
    }

    s_txLen = packet_len;
    s_txOff = 0;
    (void)tcp_tx_service(link); // opportunistic first drain
    return 0;
}

void tcp_link_close(net_link_t *link) {
    if (link != NULL && link->tcp_sock != -1) {
        shutdown(link->tcp_sock, 0);
        close(link->tcp_sock);
        link->tcp_sock = -1;
    }
    tcp_reset_streams();
}
