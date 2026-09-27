#include "xpcapi.hpp"

#include "stdio.h"

using namespace xpcapi;

char target_ip[] = "127.0.0.1";
char target_port[] = "22222";
constexpr char target_model[] = "AM2_3_1_01HWfullV2_PCIe";
constexpr char path[] = "../models";




int main(int argc, char** argv) {
    printf("Running\n");

    int port = xPCOpenTcpIpPort(target_ip, target_port);

    printf("Opened Port\n");

    xPCLoadApp(port, path, target_model);

    printf("Sent App\n");

    printf("Done\n");

}
