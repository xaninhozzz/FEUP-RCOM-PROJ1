#ifndef _SPECIAL_BYTES_H_
#define _SPECIAL_BYTES_H_

#define FLAG 0x7E
#define ESC  0x7D
#define STUFF_XOR 0x20

// Addresses
#define A_TX 0x03  
#define A_RX 0x01  

//Control field
#define SET 0x03
#define UA  0x07
#define DISC 0x0B
#define RR(r)  (0x05 | ((r) << 7))
#define REJ(r) (0x01 | ((r) << 7))

#define C(r) ((r) << 6)

#endif // _SPECIAL_BYTES_H_