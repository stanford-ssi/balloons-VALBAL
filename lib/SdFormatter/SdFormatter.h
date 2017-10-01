/*
  Stanford Student Space Initiative
  Balloons | VALBAL | September 2017
  Davy Ragland | dragland@stanford.edu

  File: SdFormatter.h
  --------------------------
  Interface to SD Card formatter.
*/

#ifndef SdFormatter_H
#define SdFormatter_H

 #include <stdint.h>

class SdFormatter {
public:
/********************************  FUNCTIONS  *********************************/
  void format();
private:
  uint8_t chipSelect = 23;
};

#endif
