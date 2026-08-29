#pragma once

// Vendored QLCP socket transport. Duplicates av-prop/Firmware/Upper_Control/wifi/;

#include <esp_err.h>
#include <esp_netif.h>
#include <netdb.h>
#include <stdint.h>
#include <stdbool.h>

#include "qlcp_lib.h"

// Non-blocking, FreeRTOS-free socket layer for the QLCP link.
// Every *_service() call is bounded (one poll / a few hundred bytes) so the
// whole layer can be driven from the main loop under the task watchdog.
// Service return convention: 1 = progress/done, 0 = nothing yet, -1 = error
// (caller closes and re-discovers).

typedef struct {
    esp_netif_t *netif_handle;
    char server_ip[IPADDR_STRLEN_MAX];
    uint16_t server_tcp_port;
    uint16_t server_udp_port;
    int32_t tcp_sock;
    int32_t udp_sock;
    int32_t discovery_sock;
} net_link_t;

void net_link_init(net_link_t *link);
void net_link_close_all(net_link_t *link);

// QLCP discovery (protocol v3): the server multicasts a QLCP DISCOVERY
// packet (header-only, type 0x01) to 239.100.0.1:10000; we learn its IP
// from the packet's source address. The socket stays open across service
// calls.
esp_err_t discovery_listen_begin(net_link_t *link);
int discovery_listen_service(net_link_t *link); // 1 = server_ip filled
void discovery_listen_end(net_link_t *link);

// TCP control channel (non-blocking connect + incremental framing).
esp_err_t tcp_connect_begin(net_link_t *link);
int tcp_connect_service(net_link_t *link); // 1 = connected
// 1 = packet decoded into out; 2 = valid framing but unrecognized PACKET_TYPE
// (out carries only packet_type + header.sequence, enough to NACK per spec 10.7)
int tcp_rx_service(net_link_t *link, qlcp_client_payload *out);
int tcp_tx_payload(net_link_t *link, const qlcp_server_payload *payload); // 0 = accepted, 1 = busy (retry next tick), -1 = error
int tcp_tx_service(net_link_t *link); // 1 = idle, 0 = still draining
void tcp_link_close(net_link_t *link);

// UDP telemetry channel (lossy by design — full buffers drop the datagram).
esp_err_t udp_create_socket(net_link_t *link);
int udp_send_data(net_link_t *link, const qlcp_data_packet *data); // 0 = sent or dropped
