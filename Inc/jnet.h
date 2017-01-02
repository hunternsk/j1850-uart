#include "stdint.h"

// Bit Length
#define TP1_MIN 21
#define TP1_NOM	24
#define TP1_MAX 28

// "1" Dominant Width
#define TP2_MIN 5
#define TP2_MAX 12

// "0" Dominant Width
#define TP3_MIN 13
#define TP3_MAX 20

// "SOF" Dominant Width
#define TP4_MIN 29
#define TP4_NOM 31
#define TP4_MAX 36

// "SOF,BRK" Length
#define TP5_MIN 45
#define TP5_NOM 48
#define TP5_MAX 52

// "BRK" Dominant Width
#define TP6_MIN 37
#define TP6_MAX 44

// "EOD" + Bit Length
#define TP7_MIN 43
#define TP7_MAX 51

// "EOF" + Bit Length
#define TP8_MIN 69
#define TP8_MAX 76

// "EOF + IFS" + Bit Length
#define TP9_MIN 86

// "0" Passive Width
#define TP10_MIN 4
#define TP10_MAX 15

// J1850 Frame 
typedef struct JFrame {
	uint8_t hdr;
	uint8_t dst;
	uint8_t src;
	uint8_t d0;
	uint8_t d1;
	uint8_t d2;
	uint8_t d3;
	uint8_t d4;
	uint8_t d5;
	uint8_t d6;
	uint8_t d7;
	uint8_t sz;
} jFrame;
