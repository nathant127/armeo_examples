#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <stdexcept>

class Connection {
public:
    Connection();
    ~Connection();

    int con_connect(const char* addr, int port);

    int con_send(const void* buf, size_t n) const;
    int con_recv(void* buf, size_t n) const;


private:
    int socket_;
    sockaddr_in server_addr_;

};
