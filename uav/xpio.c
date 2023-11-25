#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <signal.h>

#define MAX_DATA_SIZE 2048
#define MAX_DREF_SIZE 500
#define XP_FREQ 60
#define XP_FIND_TIMEOUT 1
#define NUM_RX_DATA 22
#define NUM_TX_DATA 11

int stop_flag = 0;

struct rx_data_entry {
    const char *dref;
    float value;
};

struct tx_data_entry {
    const char *dref;
    float value;
};

struct rx_data_entry rx_data[NUM_RX_DATA] = {
    {"fmuas/att/attitude_quaternion_x", 0.0},
    {"fmuas/att/attitude_quaternion_y", 0.0},
    {"fmuas/att/attitude_quaternion_z", 0.0},
    {"fmuas/att/attitude_quaternion_w", 0.0},
    {"fmuas/att/rollrate", 0.0},
    {"fmuas/att/pitchrate", 0.0},
    {"fmuas/att/yawrate", 0.0},
    {"fmuas/gps/latitude", 0.0},
    {"fmuas/gps/longitude", 0.0},
    {"fmuas/gps/altitude", 0.0},
    {"fmuas/gps/vn", 0.0},
    {"fmuas/gps/ve", 0.0},
    {"fmuas/gps/vd", 0.0},
    {"fmuas/radalt/altitude", 0.0},
    {"fmuas/adc/ias", 0.0},
    {"fmuas/adc/aoa", 0.0},
    {"sim/time/paused", 1.0},
    {"sim/time/local_date_days", 0.0},
    {"sim/time/zulu_time_sec", 0.0},
    {"fmuas/clock/time", 0.0},
    {"fmuas/camera/pitch_actual", 0.0},
    {"fmuas/camera/roll_actual", 0.0},
};

struct tx_data_entry tx_data[NUM_TX_DATA] = {
    {"fmuas/afcs/output/elevon1", 0.0},
    {"fmuas/afcs/output/elevon2", 0.0},
    {"fmuas/afcs/output/wing_tilt", 0.0},
    {"fmuas/afcs/output/wing_stow", 0.0},
    {"fmuas/afcs/output/rpm1", 0.0},
    {"fmuas/afcs/output/rpm2", 0.0},
    {"fmuas/afcs/output/rpm3", 0.0},
    {"fmuas/afcs/output/rpm4", 0.0},
    {"fmuas/camera/roll", 0.0},
    {"fmuas/camera/pitch", 180.0},
    {"fmuas/python_running", 1.0},
};

void handle_sigint(int sig) {
    stop_flag = 1;
}

void send_rx_data(int sockfd) {
    for (int i = 0; i < NUM_RX_DATA; ++i) {
        char msg[MAX_DREF_SIZE];
        snprintf(msg, sizeof(msg), "RREF%.4x%.4x%s", XP_FREQ, i, rx_data[i].dref);
        send(sockfd, msg, sizeof(msg), 0);
    }
}

void receive_data(int sockfd) {
    char buffer[MAX_DATA_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int bytes_received = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_len);

    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        char header[5];
        strncpy(header, buffer, 4);
        header[4] = '\0';

        if (strcmp(header, "RREF") == 0) {
            int num_values = (bytes_received - 5) / 8;
            int offset = 5;

            for (int i = 0; i < num_values; ++i) {
                int index = *(int *)&buffer[offset];
                float value = *(float *)&buffer[offset + sizeof(int)];
                offset += sizeof(int) + sizeof(float);

                if (index < NUM_RX_DATA) {
                    rx_data[index].value = value;
                }
            }
        }

        // Send tx_data back to X-Plane
        for (int i = 0; i < NUM_TX_DATA; ++i) {
            char msg[MAX_DREF_SIZE];
            snprintf(msg, sizeof(msg), "DREF%.4f%s", tx_data[i].value, tx_data[i].dref);
            send(sockfd, msg, sizeof(msg), 0);
        }
    }
}

int main() {
    signal(SIGINT, handle_sigint);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("Error creating socket");
        return 1;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(4900);

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Error binding socket");
        close(sockfd);
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = XP_FIND_TIMEOUT;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    send_rx_data(sockfd);

    printf("Starting socket...\n");

    while (!stop_flag) {
        receive_data(sockfd);
        usleep(1000000 / XP_FREQ);
    }

    printf("\nClosing socket\n");

    close(sockfd);

    return 0;
}
