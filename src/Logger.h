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

//#define DEBUG_CACHE

const int CACHE_SIZE = 180;
const uint32_t MAX_LOG_TIME = 15000; // microseconds

class Logger {
public:
  Logger();
  bool initialize();
  bool wipe();
  bool writeSomething(int bn, void *frame);
  bool readSomething(int bn);
  bool log(void *data, int bytes, bool retry=true);

  JankySdioCard card;

private:
  struct block_t {
    uint8_t data[512] = {0};
  } __attribute__((packed));

  int blocks_per_frame;

  block_t dangerous_sea_of_ram[CACHE_SIZE+1];
  block_t* cache;
  uint8_t to_write = 0;
  uint8_t to_insert = 0;
  uint16_t cache_offset = 0;

  uint32_t cur_block = 0;
  uint32_t limit_block = 0;
  const uint32_t MAX_SDHC_COUNT = 0xffff;
  const uint32_t RU_MASK = 0x03ff;
  enum class SDMode {Idle, Writing, BusyWait};

  uint32_t entered;

  SDMode mode = SDMode::Idle;
  SDMode prev = SDMode::Idle;

  bool findPosition();
  bool queryData(int bn);
  bool add_to_cache(void *data, int nbytes);
  uint8_t next(uint8_t idx);

  std::bitset<CACHE_SIZE> cache_avail;

};

#endif
