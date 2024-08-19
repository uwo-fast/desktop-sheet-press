#ifndef SDCARD_H
#define SDCARD_H

#include <SD.h>

bool createNewLogFile();
void logData(const char *stateName);

#endif
