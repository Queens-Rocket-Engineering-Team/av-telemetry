#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>

#include "qlcp_lib.h"
#include "wifi_tools.h"

#define UDP_TX_BUFFER_LEN 512

static const char *TAG = "UDP";

void net_link_init(net_link_t *link) {
    if (link == NULL) {
        return;
    }
    memset(link, 0, sizeof(*link));
    link->server_tcp_port = 50000;
    link->server_udp_port = 50001;
    link->tcp_sock = -1;
    link->udp_sock = -1;
    link->ssdp_sock = -1;
}

void net_link_close_all(net_link_t *link) {
    if (link == NULL) {
        return;
    }
    ssdp_listen_end(link);
    tcp_link_close(link);
    if (link->udp_sock != -1) {
        close(link->udp_sock);
        link->udp_sock = -1;
    }
}

esp_err_t udp_create_socket(net_link_t *link) {
    if (link == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char server_port_str[10] = {0};
    snprintf(server_port_str, sizeof(server_port_str), "%u", link->server_udp_port);

    esp_err_t ret = ESP_FAIL;
    int32_t sock = -1;
    link->udp_sock = -1;

    int32_t err;
    struct addrinfo hints = {0}, *res = NULL;
    // set up UDP parameters for socket
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    err = getaddrinfo(link->server_ip, server_port_str, &hints, &res);
    if (err != 0) {
        goto cleanup;
    }

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        goto cleanup;
    }

    // link udp port to server so send() can be used instead of sendto()
    // no packet is sent here unlike connect() with tcp
    err = connect(sock, res->ai_addr, res->ai_addrlen);
    if (err != 0) {
        ESP_LOGE(TAG, "connect failed: errno %d", errno);
        goto cleanup;
    }

    err = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    if (err != 0) {
        goto cleanup;
    }

    link->udp_sock = sock;
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

int udp_send_data(net_link_t *link, const qlcp_data_packet *data) {
    if (link == NULL || data == NULL || link->udp_sock < 0) {
        return -1;
    }

    static uint8_t tx_buffer[UDP_TX_BUFFER_LEN];
    size_t packet_len = sizeof(tx_buffer);

    qlcp_lib_ret ret = qlcp_encode_data(tx_buffer, &packet_len, data);
    if (ret != QLCP_OK) {
        ESP_LOGE(TAG, "QLCP err: %d", ret);
        return -1;
    }

    ssize_t n = send(link->udp_sock, tx_buffer, packet_len, MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return 0; // telemetry is lossy — drop and move on
        }
        ESP_LOGE(TAG, "send failed: errno %d", errno);
        return -1;
    }
    return 0;
}
