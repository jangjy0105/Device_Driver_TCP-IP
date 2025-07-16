#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>

#define LCD_DEVICE      "/dev/lcd_dev"
#define SERVER_PORT     9000
#define SERVER_IP       "10.10.16.143"

int lcd_fd = -1;
int sock = -1;

void handle_sigint(int sig){

        if (lcd_fd >= 0)
        {
                write(lcd_fd, "1                ", 17);
                write(lcd_fd, "2                ", 17);
                close(lcd_fd);
                printf("lcd clear\n");
        }

        if (sock >= 0) close(sock);

        exit(0);
}

int main() {
        printf("ctrl+c로 종료\n");
        struct sockaddr_in server_addr;
        char recv_buf[100];
        int read_len;
        signal(SIGINT, handle_sigint);
        // 1. 서버 접속
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { perror("socket"); exit(1); }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1)
        {
                perror("connect");
                exit(1);
        }

        // 2. LCD 디바이스 파일 열기 (쓰기 전용으로)
        lcd_fd = open(LCD_DEVICE, O_WRONLY);
        if (lcd_fd < 0)
        {
            perror("LCD 디바이스 파일 열기 실패");
            close(sock);
            exit(1);
        }

        printf("서버 연결, LCD 준비 완료. 데이터 수신을 시작합니다...\n");

        // 3. 메인 루프: 데이터 받고 -> 프로토콜에 맞춰 가공하고 -> 드라이버에 쓰기
        while ((read_len = read(sock, recv_buf, sizeof(recv_buf) - 1)) > 0)
        {
                recv_buf[read_len] = '\0';

                char temp_val[8];
                char press_val[8];
                char line1_buf[18]; // 1번 라인에 보낼 버퍼
                char line2_buf[18]; // 2번 라인에 보낼 버퍼
                // 3a. 수신된 문자열에서 온도, 압력 값만 추출
                sscanf(recv_buf, "Temperature: %s C, Pressure: %s hPa", temp_val, press_val);

                sprintf(line1_buf, "1Temp: %s C", temp_val);

                sprintf(line2_buf, "2Press: %s hPa", press_val);

                write(lcd_fd, line1_buf, strlen(line1_buf));
                write(lcd_fd, line2_buf, strlen(line2_buf));

                printf("수신: %s\n", recv_buf);
        }

        printf("서버 연결 끊김. 프로그램 종료.\n");
        handle_sigint(SIGINT);
        return 0;
}
