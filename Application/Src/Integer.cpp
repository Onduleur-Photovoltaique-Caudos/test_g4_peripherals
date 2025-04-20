/*
 * Integer.cpp
 *
 *  Created on: Jan 20, 2024
 *      Author: josel
 */

#include "Integer.h"
#include <string.h>

static char hexDigits[] = "0123456789abcdef";


Integer::Integer() {
	// TODO Auto-generated constructor stub

}

Integer::~Integer() {
	// TODO Auto-generated destructor stub
}

bool Integer::toAXn(unsigned int n, char * retBuf, int retLen, bool prefix){
	char * startError = retBuf;
	int remainingForHex = retLen;
	if (prefix) {
		if (retLen <= 3) {
			goto error_exit;
		} else {
			strcpy (retBuf, "0x");
			retBuf += 2;
			retLen -= 2;
		}
	}
	startError = retBuf;
	remainingForHex = retLen;
	unsigned char b;
	for (int i = sizeof(n)*2 -1; i >= 0; i--) {
		b = (n & (0xF << (i*4))) >> (i*4);
		if (retLen == 0){ // no space for digit
			goto error_exit;
		}
		(*retBuf++) = hexDigits[b];
		retLen--;
	}
	if (retLen == 0){ // no space final 0
		goto error_exit;
	}
	(*retBuf++) = '\0';
	return true;

	error_exit:
	for (int i = 0; i < remainingForHex-1; i++) {
		(*startError++) = '#';
	}
	(*startError++) = '\0';
	return false;
}
