/*
 * Integer.h
 *
 *  Created on: Jan 20, 2024
 *      Author: josel
 */

#ifndef SRC_INTEGER_H_
#define SRC_INTEGER_H_



class Integer {
public:
	Integer();
	virtual ~Integer();

	static bool toAXn(unsigned int i, char * retBuf, int retLen, bool prefix);
};

#endif /* SRC_INTEGER_H_ */
