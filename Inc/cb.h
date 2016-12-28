#include "stdint.h"

typedef struct cb {
	uint8_t * buffer;
	uint8_t * buffer_end;
	size_t capacity;
	size_t count;
	size_t sz;
	uint8_t * head;
	uint8_t * tail;
} cb;
