#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef struct header {
    unsigned short magic_num;
    unsigned short cmd;
    int size;
} header_t;

header_t req_header;
unsigned char* request_buf = NULL;

#define MAX_PAYLOAD_SIZE 100
typedef struct payload {
    header_t header;
    unsigned char data[MAX_PAYLOAD_SIZE];
} payload_t;

payload_t res_packet;

int main(int argc, char** argv) {



    printf("Started\n");

    int sock = socket(AF_INET, SOCK_STREAM, 0);

    printf("Made socket\n");

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(22222);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int ret = bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

    printf("Bound to 22222\n");

    ret = listen(sock, 5);

    printf("Listening for connections...\n");

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t len;
        int connection = accept(sock, (struct sockaddr*)&client_addr, &len);

        printf("Got a connection\n");

        if (connection < 0) {
            continue;
        }

        while (1) {
            
            // Read request header
            int status = recv(connection, &req_header, sizeof(req_header), 0);
            if (status == 0) {
                printf("Connection Closed\n");
                break;
            }
            if (status != 8) {
                perror("recv failed\n");
                close(connection);
                break;
            }

            // Get cmd
            unsigned short cmd = ntohs(req_header.cmd);
            printf("Recieved cmd: 0x%X size: %i\n", cmd, req_header.size);


            // Read request data
            if (req_header.size > 0) {

                request_buf = malloc(req_header.size);
                int remaining_bytes = req_header.size;
                unsigned char* buf_write = request_buf;
                while (remaining_bytes > 0) {
                    status = recv(connection, buf_write, req_header.size, 0);

                    if (status == 0) {
                        printf("Connection Closed\n");
                        break;
                    }
                    else if (status < 0) {
                        fprintf(stderr, "data_recv failed, got %i\n", status);
                        close(connection);
                        break;
                    }

                    buf_write += status;
                    remaining_bytes -= status;
                    printf("Read %i Remaining %i\n", status, remaining_bytes);
                }
            }



            // Handle response depending on cmd
            switch (cmd) {
                case 0xC83C:
                    res_packet.header = req_header;
                    res_packet.header.size = 7+sizeof(int);
     
                    int size = 7;
                    memcpy(res_packet.data, &size, sizeof(size));
                    
                    strcpy(&res_packet.data[4], "loader");
                    
                    send(connection, &res_packet, sizeof(header_t) + res_packet.header.size, 0);
                    break;

                default:
                    req_header.size = 0;
                    send(connection, &req_header, sizeof(req_header), 0);
                    break;
            }

            

            printf("Sent Response\n");

            if (request_buf) {
                free(request_buf);
                request_buf = NULL;
            }
        }


    }



}