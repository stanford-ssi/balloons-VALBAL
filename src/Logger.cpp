#ifndef LOGGER_H
#define LOGGER_H

#include "Config.h"
#include "Logger.h"
#include "Data.h"
#include <SdFat.h>

//const int MAX_BLOCK = 6135923; // Pi gigabytes
const int MAX_BLOCK = 3*20*60*60;//7617187; // CHANGE BEFORE FLIGHT TO PI GIGABYTES

bool Logger::initialize() {
  avail.set();

  if (!sd.begin(SD_CS)) {
    Serial.println("[ERROR] can't even begin pls");
    return false;
  }

  return true;
}

bool Logger::setupLogfile() {
  avail.set();

  binFile.close();
  delay(2000); // REMOVE BEFORE FLIGHT

  int32_t fno = Hardware::EEPROMReadlong(EEPROM_LOG_FILE_NUM);
  int32_t cur = Hardware::EEPROMReadlong(EEPROM_LOG_BLOCK_CUR);
  Serial.println("EEPROM fno is ");
  Serial.println(fno);
  Serial.println("EEPROM cur is ");
  Serial.println(cur);
  if (fno != 0 && cur != 0 && (MAX_BLOCK-cur) > 2000) {
    // Truncate previous file.
    int d2 = fno % 10;
    int d1 = (fno-d2)/10;
    fileName[4] = '0' + d1;
    fileName[5] = '0' + d2;
    if (sd.exists(fileName) && binFile.open(fileName, O_RDWR)) {
      uint32_t blk = cur + 1000;
      if (blk > (MAX_BLOCK-1)) blk = MAX_BLOCK - 1;
      binFile.truncate(512L * blk);
    }
    binFile.close();
    fileName[4] = '0';
    fileName[5] = '1';
  }

  if (!findFilename()) {
    Serial.println("[ERROR] ran out of filenames wtf");
    return false;
  }

  Serial.println("Writing to");
  Serial.println(fileName);

  if (!binFile.createContiguous(sd.vwd(), fileName, 512L * MAX_BLOCK)) {
    Serial.println("[ERROR] lmao");
    return false;
  }

  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    Serial.println("[ERROR] cannot create contiguous range");
    return false;
  }

  curBlock = bgnBlock;

  Hardware::EEPROMWritelong(EEPROM_LOG_BLOCK_CUR, curBlock);
  Hardware::EEPROMWritelong(EEPROM_LOG_FILE_NUM, fileNum);

  uint8_t* builtin_cache = (uint8_t*)sd.vol()->cacheClear();
  if (builtin_cache == 0) {
    Serial.println("[ERROR] could not clear cache, you should probably stop.");
    cache[0] = extra_cache;
    return false;
  }
  cache[0] = (block_t*) builtin_cache;
  for (int i=1; i<CACHE_SIZE; i++) {
    cache[i] = &extra_cache[i-1];
  }

  logOk = true;

  return true;
}

uint8_t Logger::next(uint8_t idx) {
  return (idx + 1) % CACHE_SIZE;
}

bool Logger::writeCache(bool justDoIt, int max) {
  // Write cache contents first
  if (!sd.card()->isBusy() || justDoIt) {
    int cnt;
    int tmp = to_write;
    for (cnt=0; cnt<max; cnt++) {
      if (avail[tmp]) break;
      tmp = next(tmp);
    }
    if (cnt < 1) return false;
    if (!sd.card()->writeStart(curBlock)) {
      Serial.println("[ERROR] something v wrong!");
      return true;
    }
    for (int i=0; i<cnt; i++) {
      sd.card()->writeData((uint8_t*) cache[to_write]);
      curBlock++;

      if (curBlock % 1000 == 0) {
        Hardware::EEPROMWritelong(EEPROM_LOG_BLOCK_CUR, curBlock-bgnBlock);
        Hardware::EEPROMWritelong(EEPROM_LOG_FILE_NUM, fileNum);
      }

      avail[to_write] = true;
      to_write = next(to_write);
    }
    sd.card()->writeStop();
    return false;
  }
  return true;
}
/*

static const uint8_t   EEPROM_LOG_BLOCK_START                =              108;
static const uint8_t   EEPROM_LOG_BLOCK_END                  =              112;
static const uint8_t   EEPROM_LOG_BLOCK_CUR                  =              116;
static const uint8_t   EEPROM_LOG_FILE_NUM                   =              120;*/
/* */
bool Logger::log(struct DataFrame* frame, bool sadness) {
  if (!logOk) return false;
  /*Serial.print(to_insert);
  Serial.print(",");
  Serial.print(to_write);
  Serial.print("   ");
  for (int i=0; i < CACHE_SIZE; i++) Serial.print(avail[i] ? '1' : '0');
  Serial.println();*/

  if ((MAX_BLOCK - curBlock) < 1000 && sadness) {
    setupLogfile();
  }
  writeCache(false, CACHE_SIZE);
  if (!sadness && !avail[to_insert]) {
    Serial.println("[ERROR] no sadness allowed, losing data.");
    return false; // bye, bye, data
  }

  if (!avail[to_insert]) {
    Serial.println("Ran out of cache, resorting to sadness.");
    writeCache(true, 2);
  }

  memcpy(cache[to_insert], frame, sizeof(DataFrame));
  avail[to_insert] = 0;
  to_insert = next(to_insert);

  return true; // wat
}

bool Logger::findFilename() {
  while (sd.exists(fileName)) {
    if (fileName[5] != '9') {
      fileName[5]++;
    } else {
      fileName[5] = '0';
      if (fileName[4] == '9') {
        return false;
      }
      fileName[4]++;
    }
  }
  fileNum = ((int) (fileName[5]-'0')) + ((int) (fileName[4]-'0'))*10;
  return true;
}

#endif
