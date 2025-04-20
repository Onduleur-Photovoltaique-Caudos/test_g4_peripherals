#ifndef __COMMAND_H__
#define __COMMAND_H__

void initializeCommand();

void peekProcessCommand();

void logMessage(const char * message);

void displayRecordedMeasurements();
char * my_itoa(int n, int maxVal = 100000);


#endif
