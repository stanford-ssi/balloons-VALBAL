#include <Arduino.h>
#include <SdFat.h>

SdioCardEX card;

void setup() {
  Serial.begin(115200);
  delay(2000);
  if (!card.begin()) {
    Serial.println("[SD ERROR] Could not initialize SD card.");
  }
  uint8_t block[516];
  uint32_t max = card.cardSize()-1;

  for (int k=0; k<max; k++) {
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
