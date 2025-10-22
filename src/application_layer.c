#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *roleStr, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole role = (strcmp(roleStr, "tx") == 0) ? LlTx : LlRx;

    LinkLayer connection = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
        .role = role
    };

    strcpy(connection.serialPort, serialPort);

    if (llopen(connection) < 0) {
        printf("Failed to establish connection.\n");
        fflush(stdout);
        return;
    }

    printf("Connection established successfully.\n");
    fflush(stdout);

    
    // TX: send file
    if (role == LlTx) {
        FILE *fp = fopen(filename, "rb");
        if (!fp) {
            perror("fopen");
            llclose();
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        int block = 0;

        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, fp)) > 0) {
            block++;
            int sent = llwrite(buffer, bytesRead);
            printf("[APP TX] block=%d bytesRead=%d llwrite_return=%d\n", block, bytesRead, sent);
            fflush(stdout);
            if (sent < 0) {
                printf("Error writing data.\n");
                fclose(fp);
                llclose();
                return;
            }
        }

        fclose(fp);
        printf("File sent successfully.\n");

    } else { // RX: receive file
        FILE *fp = fopen(filename, "wb");
        if (!fp) {
            perror("fopen");
            llclose();
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        int block = 0;

        // Read until llread returns 0 (EOF) or negative on error
        while (1) {
            bytesRead = llread(buffer);
            printf("[APP RX] block=%d llread_return=%d\n", ++block, bytesRead);
            fflush(stdout);
            if (bytesRead > 0) {
                fwrite(buffer, 1, bytesRead, fp);
                fflush(fp);
            } else if (bytesRead == 0) {
                // end of transmission
                break;
            } else {
                printf("Error reading data.\n");
                fclose(fp);
                llclose();
                return;
            }
        }

        fclose(fp);
        printf("File received successfully.\n");
    }

    if (llclose() == 0)
        printf("Connection closed.\n");
    else
        printf("Error closing connection.\n");
    fflush(stdout);
}