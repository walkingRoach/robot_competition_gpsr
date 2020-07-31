#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <time.h>

#define    CONNET_MAX     100

int main()
{
    int socketfd;
    char buf[CONNET_MAX] = {0};
    char input_buf[CONNET_MAX] = {0};
    struct sockaddr_in ser_addr;

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(8889);
    ser_addr.sin_addr.s_addr = inet_addr("192.168.199.177");

    socketfd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketfd < 0){
        printf("socket error!\n");
        exit(0);
    }

    if (connect(socketfd, (struct sockaddr *)&ser_addr, sizeof(ser_addr)) < 0){
        printf("connect error\n");
        printf("%d\n", errno);
        exit(0);
    }

    recv(socketfd, buf, sizeof(buf) - 1, 0);
    printf("server rec:%s\n", buf);

    send(socketfd, "Hello World!\n", 20, 0);
    send(socketfd, "yingke!\n", 20, 0);
    //while(1) {
       // gets(input_buf);
       // if (0 == strcmp(input_buf, "close")){
        //    break;
       // }

        //send(socketfd, input_buf, strlen(input_buf), 0);
       // printf("client send:%s\n", input_buf);
   //}

    //send(socketfd, "close", 6, 0);
    //exit(0);
}
