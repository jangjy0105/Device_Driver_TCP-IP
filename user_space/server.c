#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>

#define PORT            9000
#define BMP_DEVICE      "/dev/bmp180"

void * server_function (void * args) {
        int sockfd = *((int*)args);
        int fd;
        char bmp_buf[64];

        printf("Thread (%d): Client connected, start sending BMP180 data...\n", getpid());

        while(1)
        {
                int fd = open(BMP_DEVICE, O_RDONLY);
                if (fd < 0) {
                        perror("open");
                        break;
                }
                memset(bmp_buf, 0, sizeof(bmp_buf));
                ssize_t len = read(fd, bmp_buf, sizeof(bmp_buf) - 1);
                close(fd);

                if (len <= 0)
                {
                        perror("Failed to read from BMP180");
                        break;
                }
                if(send(sockfd, bmp_buf, strlen(bmp_buf), 0) == -1)
                {
                        perror("Send failed or client disconnected");
                        break;
                }
                printf("송신: %s\n", bmp_buf);
                sleep(1);
        }

        printf("Thread (%d): Client disconnected.\n", getpid());
        close(sockfd);
        return NULL;
}

int main(void){
        signal(SIGPIPE, SIG_IGN);
        int server_sfd, client_sfd;
        struct sockaddr_in server_addr;
        struct sockaddr_in client_addr;
        int sock_size;
        int yes = 1;
        int i;
        pthread_t tid;

        if((server_sfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
                perror("socket() error");
                exit(1);
        }

        if(setsockopt(server_sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1){
                perror("setsockopt() error");
                exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(PORT); //server port number setting
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        memset(&(server_addr.sin_zero), '\0', 8);

        //server ip & port number setting
        if(bind(server_sfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1){
                perror("bind() error");
                exit(1);
        }

        //client backlog setting
        if(listen(server_sfd, 5) == -1){
                perror("listen() error");
                exit(1);
        }

        while(1){
                sock_size = sizeof(struct sockaddr_in);

                //wait for client request
                if((client_sfd = accept(server_sfd, (struct sockaddr *) &client_addr, &sock_size)) == -1){
                        perror("accept() error");
                        continue;
                }
                int *pclient = malloc(sizeof(int));
                *pclient = client_sfd;
                pthread_create(&tid, NULL, server_function, pclient);
        }
        return 0;
}
