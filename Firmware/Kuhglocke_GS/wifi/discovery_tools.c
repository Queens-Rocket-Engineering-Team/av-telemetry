#include "wifi_tools.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>

#include <qlcp_lib.h>

#define DISCOVERY_PORT "10000"
#define DISCOVERY_IP "239.100.0.1"
#define DISCOVERY_ANY_IP "0.0.0.0"

// Bound per service call: at most this many datagrams are drained per tick.
#define DISCOVERY_RECV_PER_TICK 2

static const char *TAG = "DISCOVERY";

esp_err_t discovery_listen_begin(net_link_t *link) {
    if (link == NULL || link->netif_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (link->discovery_sock != -1) {
        return ESP_OK; // already listening
    }

    esp_err_t ret = ESP_FAIL;
    int32_t sock = -1;

    int32_t err;
    struct addrinfo hints = {0}, *res = NULL;
    // set up UDP parameters for socket
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    err = getaddrinfo(DISCOVERY_ANY_IP, DISCOVERY_PORT, &hints, &res);
    if (err != 0) {
        goto cleanup;
    }

    sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        goto cleanup;
    }

    int32_t enable = 1;
    err = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof enable);
    if (err != 0) {
        goto cleanup;
    }

    // bind the socket to the discovery port, any ip
    err = bind(sock, res->ai_addr, res->ai_addrlen);
    if (err != 0) {
        goto cleanup;
    }

    // add membership to the QLCP discovery multicast group
    esp_netif_ip_info_t ip_info = {0};
    esp_netif_get_ip_info(link->netif_handle, &ip_info);

    struct in_addr local_addr;
    inet_addr_from_ip4addr(&local_addr, &ip_info.ip);

    ip_mreq imreq = {0};
    imreq.imr_interface.s_addr = local_addr.s_addr;
    err = inet_pton(AF_INET, DISCOVERY_IP, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        goto cleanup;
    }
    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof imreq);
    if (err != 0) {
        goto cleanup;
    }

    err = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    if (err != 0) {
        goto cleanup;
    }

    link->discovery_sock = sock;
    ESP_LOGI(TAG, "Listening for server discovery");
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

int discovery_listen_service(net_link_t *link) {
    if (link == NULL || link->discovery_sock < 0) {
        return -1;
    }

    static uint8_t buffer[QLCP_HEADER_SIZE];
    struct sockaddr_in remote_addr = {0};
    socklen_t remote_addr_len;

    for (int i = 0; i < DISCOVERY_RECV_PER_TICK; i++) {
        remote_addr_len = sizeof remote_addr;
        ssize_t len = recvfrom(link->discovery_sock, buffer, sizeof buffer, MSG_DONTWAIT,
                               (struct sockaddr *)&remote_addr, &remote_addr_len);
        if (len < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return 0;
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            return -1;
        }

        // Validate framing (magic num, version, length) and packet type
        // before trusting the datagram's source address as the server IP —
        // the multicast group is world-writable.
        qlcp_client_payload payload = {0};
        if (qlcp_decode_server_to_client(&payload, buffer, (size_t)len) != QLCP_OK) {
            continue;
        }
        if (payload.packet_type != QLCP_PT_DISCOVERY) {
            continue;
        }

        inet_ntop(remote_addr.sin_family, &remote_addr.sin_addr,
                  link->server_ip, sizeof(link->server_ip));
        ESP_LOGI(TAG, "Server discovered at %s", link->server_ip);
        return 1;
    }
    return 0;
}

void discovery_listen_end(net_link_t *link) {
    if (link != NULL && link->discovery_sock != -1) {
        close(link->discovery_sock);
        link->discovery_sock = -1;
    }
}
