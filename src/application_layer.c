#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

// Application-layer packet types and TLVs
#define APP_START  0x01
#define APP_DATA   0x02
#define APP_END    0x03

#define T_FILESIZE 0x00
#define T_FILENAME 0x01

// Compile-time selectable application data chunk size for DATA packets.
// Default uses the full allowed payload minus the 4-byte data header (C,Seq,L2,L1).
// To change at compile time, define APP_DATA_CHUNK (e.g., add -DAPP_DATA_CHUNK=200 when compiling)
// or edit the define below. This mirrors older project styles that fix the application chunk size
// at compile-time instead of runtime.
#ifndef APP_DATA_CHUNK
#define APP_DATA_CHUNK (MAX_PAYLOAD_SIZE - 4)
#endif

// Build START/END control packet with TLVs: filesize and filename
static int build_control_packet(uint8_t ctype, const char *fname, off_t fsize, unsigned char *out, int max)
{
    if (!out || max <= 0) return -1;
    int pos = 0;
    if (pos + 1 > max) return -1;
    out[pos++] = ctype;

    // T=filesize
    uint8_t sizeBytes[8];
    int sb = 0;
    {
        uint64_t sz = (uint64_t)fsize;
        int started = 0;
        for (int i = 7; i >= 0; i--) {
            uint8_t b = (sz >> (i * 8)) & 0xFF;
            if (!started && b == 0 && i != 0) continue;
            started = 1;
            sizeBytes[sb++] = b;
        }
        if (sb == 0) { sizeBytes[sb++] = 0; }
    }
    if (pos + 2 + sb > max) return -1;
    out[pos++] = T_FILESIZE;
    out[pos++] = (uint8_t)sb;
    memcpy(out + pos, sizeBytes, sb);
    pos += sb;

    size_t nameLen = fname ? strlen(fname) : 0;
    if (nameLen > 255) nameLen = 255;

    if (pos + 2 + (int)nameLen > max) return -1;
    out[pos++] = T_FILENAME;           // T
    out[pos++] = (uint8_t)nameLen;     // L
    if (nameLen > 0) {
        memcpy(out + pos, fname, nameLen); // V
        pos += (int)nameLen;
    }

    return pos;
}

// Build DATA packet
static int build_data_packet(uint8_t seq, const unsigned char *data, int len, unsigned char *out, int max)
{
    if (!out || max < 4 || len < 0) return -1;
    int header = 4;
    if (header + len > max) return -1;
    out[0] = APP_DATA;
    out[1] = seq;
    out[2] = (uint8_t)((len >> 8) & 0xFF);
    out[3] = (uint8_t)(len & 0xFF);
    if (len > 0) memcpy(out + header, data, len);
    return header + len;
}

// Parse START/END control packet; returns 0 on success
static int parse_control_packet(const unsigned char *buf, int len, char *out_name, size_t name_cap, off_t *out_size)
{
    if (!buf || len < 1) return -1;
    int pos = 1; // skip C
    uint64_t fsize = 0;
    int haveSize = 0;

    while (pos + 2 <= len) {
        uint8_t T = buf[pos++];
        uint8_t L = buf[pos++];
        if (pos + L > len) return -1;

        if (T == T_FILESIZE) {
            if (L == 0 || L > 8) return -1;
            fsize = 0;
            for (int i = 0; i < L; ++i) {
                fsize = (fsize << 8) | buf[pos + i];
            }
            haveSize = 1;
        } else if (T == T_FILENAME) {
            size_t copy = (size_t)L;
            if (copy >= name_cap) copy = name_cap ? name_cap - 1 : 0;
            if (copy > 0 && out_name) {
                memcpy(out_name, buf + pos, copy);
                out_name[copy] = '\0';
            } else if (L == 0) {
                if (out_name && name_cap > 0) out_name[0] = '\0';
            }
        }
        pos += L;
    }

    if (out_size) *out_size = (off_t)fsize;
    return (haveSize ? 0 : -1);
}

void applicationLayer(const char *serialPort, const char *roleStr, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayerRole role = (strcmp(roleStr, "tx") == 0) ? LlTx : LlRx;

    LinkLayer connection = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
        .role = role
    };

    // serialPort is an array so we need to copy it, we can't assign it 
    strcpy(connection.serialPort, serialPort);

    if (llopen(connection) < 0) {
        printf("Failed to establish connection.\n");
        return;
    }

    printf("Connection established successfully.\n");

    if (role == LlTx) {
        // TX: open file and get its size
        FILE *fp = fopen(filename, "rb");
        if (!fp) {
            perror("fopen");
            llclose();
            return;
        }
        struct stat st;
        if (stat(filename, &st) != 0) {
            perror("stat");
            fclose(fp);
            llclose();
            return;
        }
        off_t fsize = st.st_size;

        // Send START control packet, build packet and call llwrite
        unsigned char ctrl[MAX_PAYLOAD_SIZE];
        int clen = build_control_packet(APP_START, filename, fsize, ctrl, sizeof(ctrl));
        if (clen < 0) {
            printf("Failed to build START control packet.\n");
            fclose(fp);
            llclose();
            return;
        }
        if (llwrite(ctrl, clen) < 0) {
            printf("Failed to send START control packet.\n");
            fclose(fp);
            llclose();
            return;
        }

        // Send DATA packets
        unsigned char fileBuf[MAX_PAYLOAD_SIZE];
        unsigned char pkt[MAX_PAYLOAD_SIZE];
        uint8_t seq = 0;
        size_t totalSent = 0;
        const int maxDataPerPkt = APP_DATA_CHUNK; // compile-time fixed chunk size
        printf("[APP TX] Using chunk size (compile-time): %d bytes\n", maxDataPerPkt);

        while (1) {
            int toRead = maxDataPerPkt;
            int bytesRead = fread(fileBuf, 1, toRead, fp);
            if (bytesRead < 0) bytesRead = 0;
            if (bytesRead == 0) break;

            int plen = build_data_packet(seq, fileBuf, bytesRead, pkt, sizeof(pkt));
            if (plen < 0) {
                printf("Failed to build DATA packet.\n");
                fclose(fp);
                llclose();
                return;
            }

            int sent = llwrite(pkt, plen);
            printf("[APP TX] seq=%u chunk=%d llwrite_return=%d\n", seq, bytesRead, sent);
            if (sent < 0) {
                printf("Error writing data.\n");
                fclose(fp);
                llclose();
                return;
            }

            totalSent += bytesRead;
            seq = (uint8_t)((seq + 1) & 0xFF);
        }

        // Send END control packet
        int elen = build_control_packet(APP_END, filename, fsize, ctrl, sizeof(ctrl));
        if (elen < 0) {
            printf("Failed to build END control packet.\n");
            fclose(fp);
            llclose();
            return;
        }
        if (llwrite(ctrl, elen) < 0) {
            printf("Failed to send END control packet.\n");
            fclose(fp);
            llclose();
            return;
        }

        fclose(fp);
        printf("File sent successfully.\n");

    } else {
        // RX: wait for START, extract metadata, then receive DATA until END
        unsigned char buf[MAX_PAYLOAD_SIZE];
        int bytesRead;
        int gotStart = 0, gotEnd = 0;
        off_t expectedSize = 0;
        off_t written = 0;
        char remoteName[256] = {0};
        FILE *fp = NULL;

        while (!gotEnd) {
            bytesRead = llread(buf);
            if (bytesRead < 0) {
                printf("Error reading data.\n");
                if (fp) fclose(fp);
                llclose();
                return;
            }
            if (bytesRead == 0) {
                // Ignore empty packets (shouldn't happen)
                continue;
            }

            uint8_t ctype = buf[0];

            if (ctype == APP_START) {
                if (gotStart) {
                    // Duplicate START, ignore
                    continue;
                }
                if (parse_control_packet(buf, bytesRead, remoteName, sizeof(remoteName), &expectedSize) != 0) {
                    printf("Malformed START packet.\n");
                    llclose();
                    return;
                }
                const char *outName = (filename && filename[0]) ? filename : remoteName;
                if (!outName || outName[0] == '\0') outName = "received_file";

                fp = fopen(outName, "wb");
                if (!fp) {
                    perror("fopen");
                    llclose();
                    return;
                }
                gotStart = 1;
                printf("[APP RX] START name=\"%s\" size=%lld\n", outName, (long long)expectedSize);
            }
            else if (ctype == APP_DATA) {
                if (!gotStart) {
                    // Ignore data before START
                    continue;
                }
                if (bytesRead < 4) {
                    printf("Malformed DATA header.\n");
                    if (fp) fclose(fp);
                    llclose();
                    return;
                }
                uint8_t seq = buf[1];
                int len = ((int)buf[2] << 8) | buf[3];
                if (4 + len > bytesRead || len < 0) {
                    printf("Malformed DATA length.\n");
                    if (fp) fclose(fp);
                    llclose();
                    return;
                }
                if (len > 0 && fp) {
                    size_t w = fwrite(buf + 4, 1, (size_t)len, fp);
                    if (w != (size_t)len) {
                        perror("fwrite");
                        fclose(fp);
                        llclose();
                        return;
                    }
                    written += len;
                }
                printf("[APP RX] DATA seq=%u len=%d total_written=%lld\n", seq, len, (long long)written);
            }
            else if (ctype == APP_END) {
                // Optionally verify END metadata
                off_t endSize = 0;
                char endName[256];
                if (parse_control_packet(buf, bytesRead, endName, sizeof(endName), &endSize) == 0) {
                    if (expectedSize && endSize && endSize != expectedSize) {
                        printf("Warning: END size mismatch (%lld vs %lld)\n", (long long)expectedSize, (long long)endSize);
                    }
                }
                gotEnd = 1;
            }
            else {
                // Unknown packet, ignore
            }
        }

        if (fp) fclose(fp);
        printf("File received successfully (%lld bytes).\n", (long long)written);
    }

    if (llclose() == 0)
        printf("Connection closed.\n");
    else
        printf("Error closing connection.\n");
}