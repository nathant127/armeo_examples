#include "xpc_comms.hpp"

#include <sys/time.h>

Connection::Connection() {
    socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ == -1) {
        throw std::runtime_error("Failed to create unix socket");
    }
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv)) < 0) {
        throw std::runtime_error("Failed to set socket timeout");
    }
}
Connection::~Connection() {
    close(socket_);
}

int Connection::con_connect(const char* addr, int port) {
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    server_addr_.sin_addr.s_addr = inet_addr(addr);

    int ret;
    for (int i = 0; i < 10; i++) {
        ret = connect(socket_, reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));
        if (ret != -1) {
            break;

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (ret == -1) {
        throw std::runtime_error{"Failed to connect"};
    }





    // TODO: Handle connection messages, i think it pings or something?

    return 0;
}

int Connection::con_send(const void* buf, size_t n) const {
    return send(socket_, buf, n, 0);
}

int Connection::con_recv(void* buf, size_t n) const {
    void* buf_write = buf;
    size_t bytes_to_read = n;

    while (bytes_to_read > 0) {
        int ret = recv(socket_, buf_write, bytes_to_read, 0);
        if (ret <= 0) {
            return ret;
        }
        buf_write += ret;
        bytes_to_read -= ret;
    }

    return n;
}