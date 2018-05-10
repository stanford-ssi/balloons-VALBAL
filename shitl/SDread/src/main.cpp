#include <Arduino.h>
#include <SdFat.h>

SdioCardEX card;

void setup() {
  Serial.begin(115200);
  delay(2000);
  if (!card.begin()) {
    Serial.println("[SD ERROR] Could not initialize SD card.");
  }
  Serial.println("ye");
  while (Serial.read() != 0xaa);
  while (!Serial.available()); uint32_t b0 = Serial.read();
  while (!Serial.available()); uint32_t b1 = Serial.read();
  while (!Serial.available()); uint32_t b2 = Serial.read();
  while (!Serial.available()); uint32_t b3 = Serial.read();

  uint32_t k = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

  uint8_t block[516];
  uint32_t max = card.cardSize()-1;

  for (; k<max; k++) {
    card.readBlock(k, block);

    bool stop = true;

    uint32_t chk = 0;
    for (int i=0; i<128; i++) {
      uint32_t b = ((uint32_t*)block)[i];
      chk += i*b;
      if (b != 0xffffffff && b != 0x0) {
        stop = false;
      }
    }
    memcpy((int*)(block+512), &chk, 4);

    if (stop) break;

    Serial.write(block, 516);

  }

  Serial.println("Ok, transfer done");
}

void loop() {
    // put your main code here, to run repeatedly:
}
