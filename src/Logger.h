/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Joan Creus-Costa | jcreus@stanford.edu

  File: Logger.h
  --------------------------
  Interface to commpressed logging utility.
*/

#ifndef VAL_LOGGER_H
#define VAL_LOGGER_H

#include <SdFat.h>
#include <SdFormatter.h>
#include "Hardware.h"
#include <bitset>

struct block_t {
  uint8_t data[512] = {0};
} __attribute__((packed));

const int CACHE_SIZE = 24;

class Logger {
public:
/**********************************  SETUP  ***********************************/
  bool format();
  bool initialize();

/********************************  FUNCTIONS  *********************************/
  bool log(struct DataFrame *frame, bool sadness);
  bool setupLogfile();

private:
/*********************************  OBJECTS  **********************************/
  SdFat sd;
  File binFile;

  char fileName[14] = "data01.bin"; // FORTRAN style

  uint32_t bgnBlock, endBlock;
  uint32_t curBlock;

  block_t extra_cache[CACHE_SIZE-1];
  block_t* cache[CACHE_SIZE] = {NULL};

  bool findFilename();

  int fileNum;

  void operator=(const Logger& rhs) = delete;

  std::bitset<CACHE_SIZE> avail;

  uint8_t to_insert = 0;
  uint8_t to_write = 0;

  uint8_t next(uint8_t idx);
  bool writeCache(bool weep, int max);

  bool logOk = false;
  SdFormatter formatter;
};

#endif
