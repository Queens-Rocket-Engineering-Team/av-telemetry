#include "wifi_tools.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>

#define SSDP_PORT "1900"
#define SSDP_IP "239.255.255.250"
#define SSDP_ANY_IP "0.0.0.0"

// Bound per service call: at most this many datagrams are drained per tick.
#define SSDP_RECV_PER_TICK 2

static const char *TAG = "SSDP";

esp_err_t ssdp_listen_begin(net_link_t *link) {
    if (link == NULL || link->netif_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (link->ssdp_sock != -1) {
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

    err = getaddrinfo(SSDP_ANY_IP, SSDP_PORT, &hints, &res);
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

    // bind the socket to port 1900, any ip
    err = bind(sock, res->ai_addr, res->ai_addrlen);
    if (err != 0) {
        goto cleanup;
    }

    // add membership to SSDP multicast ip
    esp_netif_ip_info_t ip_info = {0};
    esp_netif_get_ip_info(link->netif_handle, &ip_info);

    struct in_addr local_addr;
    inet_addr_from_ip4addr(&local_addr, &ip_info.ip);

    ip_mreq imreq = {0};
    imreq.imr_interface.s_addr = local_addr.s_addr;
    err = inet_pton(AF_INET, SSDP_IP, &imreq.imr_multiaddr.s_addr);
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

    link->ssdp_sock = sock;
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

int ssdp_listen_service(net_link_t *link) {
    if (link == NULL || link->ssdp_sock < 0) {
        return -1;
    }

    static char buffer[1024];
    struct sockaddr_in remote_addr = {0};
    socklen_t remote_addr_len;

    for (int i = 0; i < SSDP_RECV_PER_TICK; i++) {
        remote_addr_len = sizeof remote_addr;
        ssize_t len = recvfrom(link->ssdp_sock, buffer, sizeof buffer - 1, MSG_DONTWAIT,
                               (struct sockaddr *)&remote_addr, &remote_addr_len);
        if (len < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return 0;
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            return -1;
        }

        buffer[len] = '\0'; // recvfrom does not null terminate buffer
        ESP_LOGD(TAG, "%s", buffer);

        // check if received data matches server SSDP request
        if (strcasestr(buffer, "M-SEARCH * HTTP/1.1") != NULL &&
            strcasestr(buffer, "HOST: 239.255.255.250:1900") != NULL &&
            strcasestr(buffer, "ST: urn:qretprop:espdevice:1") != NULL) {
            inet_ntop(remote_addr.sin_family, &remote_addr.sin_addr,
                      link->server_ip, sizeof(link->server_ip));
            ESP_LOGI(TAG, "Server discovered at %s", link->server_ip);
            return 1;
        }
    }
    return 0;
}

void ssdp_listen_end(net_link_t *link) {
    if (link != NULL && link->ssdp_sock != -1) {
        close(link->ssdp_sock);
        link->ssdp_sock = -1;
    }
}
