/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Joan Creus-Costa | jcreus@stanford.edu

  File: Logger.cpp
  --------------------------
  Implementation of Logger.h
*/

#ifndef LOGGER_H
#define LOGGER_H

#include "Config.h"
#include "Logger.h"
#include "Data.h"

//#define DEBUG_CACHE

//#define Serial Serial2

Logger::Logger() {
  cache_avail.set();
}

bool Logger::log(void *data, int nbytes, bool retry) {
  /*Serial.print(cur_block);
  Serial.print(" ");
    Serial.print(to_insert);
    Serial.print(" ");
    Serial.print(to_write);
    Serial.print(" ");*/
  bool added = add_to_cache(data, nbytes);
  if (!added) Serial.println("DID NOT FIT :siren:");
  int dist = (to_insert-to_write+CACHE_SIZE) % CACHE_SIZE;
  /*if (dist > 2) {
    Serial.print("Distance: ");
    Serial.println(dist);
  }*/
  uint32_t t0 = micros();

  bool result;
  bool done = false;

	bool should_write = 4*dist > 3*CACHE_SIZE;
	/*Serial.print(dist);
	Serial.print(" ");
	Serial.println(CACHE_SIZE);*/

	if (should_write && mode == SDMode::Idle) {
		Serial.println("time to git going");
		limit_block = (cur_block + MAX_SDHC_COUNT) & ~RU_MASK;
		if (!card.writeStart(cur_block, limit_block - cur_block)) {
			Serial.println("[SD ERROR] writeStart failed");
			return false;
		}
		mode = SDMode::Writing;
		num_to_write = dist;
		written = 0;
	}

  while ((micros() - t0) < MAX_LOG_TIME) {
    switch (mode) {
    case SDMode::Idle:
      done = true;
      break;
    case SDMode::Writing:
      if (cache_avail[to_write]) {
        done = true;
				Serial.println("Should never reach here");
				mode = SDMode::Idle;
				card.writeStop();
        break;
      }
      if (!card.writeDataStart((const uint8_t*)&cache[to_write])) {
        Serial.println("[SD ERROR] writeBlockStart failed");
      }
      cache_avail[to_write] = true;
      to_write = next(to_write);
      mode = SDMode::BusyWait;
      entered = micros();
      cur_block++;
      break;
    case SDMode::BusyWait:
      if (card.checkWriteComplete(result)) {
        if (!result) {
          Serial.println("[SD ERROR] Block write failed!");
          return false;
        } else {
					//Serial.println("aye wrote");
					written++;
				}
        if (cur_block >= limit_block || written == num_to_write) {
	        if (!card.writeStop()) {
	          Serial.println("[SD ERROR] spooky writeStop failed");
	        } else {
						Serial.println("should go to lower power now!");
					}
					mode = SDMode::Idle;
        } else {
          mode = SDMode::Writing;
        }
      }
      break;
    }
    prev = mode;
    if (done) break;
  }

  if (!added && retry) {
    log(data, nbytes, false);
  }

  return true;
}

bool Logger::add_to_cache(void *buf, int nbytes) {
  uint8_t* data = (uint8_t*)buf;
  int max_iter = CACHE_SIZE;
  int nbytes_test = nbytes - min(512-cache_offset, nbytes);
  int to_insert_test = next(to_insert);
  while (max_iter--) {
    if (nbytes_test <= 0) {
      break;
    }
    if (!cache_avail[to_insert_test]) {
      return false;
    }
    nbytes_test -= 512;
    to_insert_test = next(to_insert_test);
  }
  // ok, it fits
  int num = min(512-cache_offset, nbytes);
  memcpy(((uint8_t*)(&cache[to_insert]))+cache_offset, data, num);
  cache_offset += num;
  data += num;
  nbytes -= num;

  if (cache_offset == 512) {
    cache_offset = 0;
    cache_avail[to_insert] = false;
    to_insert = next(to_insert);
  }

  max_iter = CACHE_SIZE;
  while (max_iter--) {
    if (nbytes == 0) {
      break;
    }
    num = min(512, nbytes);
    memcpy(&cache[to_insert], data, num);
    data += num;
    nbytes -= num;
    cache_offset += num;
    if (cache_offset == 512) {
      cache_offset = 0;
      cache_avail[to_insert] = false;
      to_insert = next(to_insert);
    }
  }
  #ifdef DEBUG_CACHE
    Serial.println("It fits!");
    Serial.print("to_write: ");
    Serial.println(to_write);
    Serial.print("to_insert: ");
    Serial.println(to_insert);
    Serial.print("cache_offset: ");
    Serial.println(cache_offset);
    Serial.print("Cache: ");
    for (int i=0; i<CACHE_SIZE; i++) Serial.print(cache_avail[i], DEC);
    Serial.println();
    Serial.println("-------");
  #endif

  return true;
}

uint8_t Logger::next(uint8_t idx) {
  return (idx + 1) % CACHE_SIZE;
}

bool Logger::initialize() {
  const int boundary_of_death = 0x20000000;
  int spooky_offset = (boundary_of_death-(int)(&dangerous_sea_of_ram)) >> 9;
  cache = (block_t*)(boundary_of_death - (spooky_offset << 9));
  Serial.println("card begin");
  if (!card.begin()) {
    Serial.println("[SD ERROR] Could not initialize SD card.");
    return false;
  }
    Serial.println("card find position");
  return findPosition();
}

bool Logger::findPosition() {
  uint32_t t0 = micros();
  uint32_t l = 0;
  uint32_t r = card.cardSize()-1;

  if (queryData(0)) {
    cur_block = 0;
    return true;
  }

  int max = 32;
  while (max--) {
    if (l > r) {
      Serial.println("wtf");
      return false;
    }
    if (l == r) {
      cur_block = l;
      Serial.println("yay found");
      Serial.println(l);
      return true;
    }
    Serial.print("[SD BS] ");
    Serial.print(l);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" -> ");
    uint32_t m = (l+r)/2;
    Serial.println(m);

    bool avail = queryData(m);
    if (!avail) {
      l = m+1;
    } else {
      r = m;
    }
  }
  uint32_t dt = micros()-t0;
  Serial.print("Took ");
  Serial.print(dt);
  Serial.println(" us");
  Serial.print(32-max);
  delay(2000);
  return false;
}

bool Logger::wipe() {
  uint32_t const ERASE_SIZE = 8192L;
  uint32_t firstBlock = 0;
  uint32_t lastBlock;
  uint32_t cardSizeBlocks = card.cardSize();
  Serial.println("card size blocks was");
  Serial.println(cardSizeBlocks);

  do {
    lastBlock = firstBlock + ERASE_SIZE - 1;
    if (lastBlock >= cardSizeBlocks) {
      lastBlock = cardSizeBlocks - 1;
    }
    Serial.print(firstBlock);
    Serial.print(" -> ");
    Serial.println(lastBlock);
    if (!card.erase(firstBlock, lastBlock)) {
      Serial.println("[SD ERROR] Could not wipe.");
      return false;
    }
    firstBlock += ERASE_SIZE;
  } while (firstBlock < cardSizeBlocks);

  uint8_t cache[512];
  if (!card.readBlock(0, cache)) {
    Serial.println("couldn't read block 0");
    return false;
  }
  if (!card.readBlock(555, cache)) {
    Serial.println("couldn't read block 555");
    return false;
  }
  Serial.println("Data set to");
  Serial.println(int(cache[0]));

  cur_block = 0;
  return true;
}

bool Logger::writeSomething(int bn, void* frame) {
  //const uint8_t stuff[512] = "hi this is a test of me writing stuff and good shit let's try this";
  if (!card.writeBlock(bn, (uint8_t*)frame)) {
      Serial.print("writeblock failed ");
      Serial.println(bn);
      Serial.println(card.errorCode());
      Serial.println(card.errorData());
  }
  return true;
}
bool Logger::readSomething(int bn) {
  char data[512];
  if (!card.readBlock(bn, (uint8_t*)data)) {
      Serial.print("[SD ERROR] readblock failed ");
      Serial.println(bn);
      Serial.println(card.errorCode());
      Serial.println(card.errorData());
  }
  for (int i=0; i<512; i++) {
    Serial.print(data[i], DEC);
    Serial.print(" ");
  }
  Serial.println();
  return true;
}

bool Logger::queryData(int bn) {
  char data[512];
  if (!card.readBlock(bn, (uint8_t*)data)) {
      Serial.print("[SD ERROR] readblock failed ");
      Serial.println(bn);
      Serial.println(card.errorCode());
      Serial.println(card.errorData());
  }
  for (int i=0; i<128; i++) {
    uint32_t b = ((uint32_t*)data)[i];
    if (b != 0xffffffff && b != 0x0) {
      return false;
    }
  }
  return true;
}

#endif
