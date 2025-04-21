#include <string.h>
#include <cstdio>

#include "Command.h"
#include "CommandPrivate.h"
#include "Measure.h"
#include "Serial.h"

#define SERIAL_BUFFER_SIZE 50
static char bufferOutConsole[SERIAL_BUFFER_SIZE];
static char bufferInConsole[SERIAL_BUFFER_SIZE];
static SerialOutput SerialOutToConsole(&huart2, bufferOutConsole, SERIAL_BUFFER_SIZE); // via USB
static SerialInput SerialInFromConsole(&huart2, bufferInConsole, SERIAL_BUFFER_SIZE);
static SerialOutput* pSerialOutToConsole;
static SerialInput* pSerialInFromConsole;

typedef enum _runState {
  eStop,
  eStopEmergency,
  eDelayed,
  eRun
} RunState;

volatile RunState runState;

volatile bool stateACWanted = true; // want to produce AC waveform

volatile bool stateAC = true; // producing AC waveform

volatile unsigned long long runDelayTimerStartTick;

void initializeCommand() {
  pSerialOutToConsole = &SerialOutToConsole;
  pSerialInFromConsole = &SerialInFromConsole;
  pSerialInFromConsole->initialize(pSerialOutToConsole);
  pSerialOutToConsole->puts("\r\nCommand Processor Ready\r\n");
}

commandAndValue getCommand() {
  commandAndValue result(none, 0);
  char strConsole[SERIAL_BUFFER_SIZE];
  char state = '\0';
  command = none;
  bool hasValue = false;
  int value = 0;
  const char commandList[] = "srazdtvf";
  char *commandListPtr;
  const char numbers12[] = "12";
  const char numbers[] = "0123456789";
  const char compositeCommandList[] = "zdtv";
  bool negativeValue = false;

  if (pSerialInFromConsole->fgetsNonBlocking(strConsole, 48)) {
    int len = strlen(strConsole);
    bool equalFound = false;
    int i;

    for (i = 0; i < len; i++) { // skip initial spaces
      char c = strConsole[i];
      if (!(c == ' ' || c == '\t' || c == '\r' || c == '\n')) {
        break;
      }
    }

    for (; i < len; i++) {
      char c = strConsole[i];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        break;
      } else if (state == '\0') {
        if ((commandListPtr = strchr(commandList, c))) { // look for first char of command
          state = c;
        }
      } else if (equalFound) {
        char *where;
        if ((where = strchr(numbers, c))) {
          value = value * 10 + where - numbers;
        } else if (c == '-') {
          negativeValue = true;
        }
      } else if (command != none) {
        if (c == '=') {
          equalFound = true;
        }
      } else if (strchr(commandList, state)) { // if we have a non 0 state ie. second command char, identify command
        char commandInput[3];
        commandInput[0] = state;
        commandInput[1] = c;
        commandInput[2] = 0;
        for (int j = 0; j < firstCompositeCommand; j++) {
          if (0 == strcmp(commandInput, simpleCommandListStr[j])) {
            command = commandArray[j];
            continue;
          }
        }
        char *where = strchr(numbers12, c);
        int indexIncrement = where - numbers12;
        if ((where = strchr(numbers12, c))) {
          command = commandArray[firstCompositeCommand + (strchr(compositeCommandList, state) - compositeCommandList) * 2 + indexIncrement];
        }
      }
    }
    // end loop, we should have a command
    if (equalFound) {
      result = commandAndValue(command, (negativeValue ? -value : value));
    }
  }
  return result;
}

void processCommand(commandAndValue cv) {
  switch (cv.command) {
    case st: // display state
      statusDisplay(cv.value == 0);
      break;

    case sm:
      measurementsDisplay(cv.value < 10, cv.value < 10 ? cv.value : cv.value - 10);
      break;

    case none:
    default:
      ;
  }
}

void peekProcessCommand() {
  commandAndValue cv = getCommand();
  if (cv.command == none) {
    return;
  }
  processCommand(cv);
}

void logMessage(const char *message) {
  pSerialOutToConsole->puts(message);
}

void sendSerial(const char* message){
pSerialOutToConsole->puts(message);
}
void statusDisplay(bool bHeader){
if (bHeader) {
pSerialOutToConsole->puts("    rt\t    z1\t    d1\t    r1\t    v1\t     z2\t    d2\t    r2\t    v2\n\r");
}
//pSerialOutToConsole->puts(my_itoa(_rt));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countZ1 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countD1 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_base / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countT1 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(getRatioV225()*400));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countZ2 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countD2 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa((PERIOD_SWITCH - _base) / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(_countT2 / COUNT_PER_NS));
//pSerialOutToConsole->puts("\t");
//pSerialOutToConsole->puts(my_itoa(getRatioV175() * 400));
//pSerialOutToConsole->puts("\n\r");
}

void measurementsDisplay(int val1, int val2) {
	char message[30];
	snprintf(message, 30, "%4d, %12d\r\n", 0, statsBuffer[0]);
	pSerialOutToConsole->puts(message);
	for (int i = 1; i < 4095; i++)
#include <cstdio>
	{
		if (statsBuffer[i] != 0)
		{
			snprintf(message, 30, "%4d, %12d\r\n", i, statsBuffer[i]);
			pSerialOutToConsole->puts(message);
		}
	}
	snprintf(message, 30, "%4d, %12d\r\n", 4095, statsBuffer[4095]);
	pSerialOutToConsole->puts(message);
}
