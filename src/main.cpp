/*
  Stanford Student Space Initiative
  Balloons | VALBAL | July 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | atedjara@stanford.edu
  Joan Creus-Costa | jcreus@stanford.edu
  John Dean | deanjl@stanford.edu
  Jonathan Zwiebel | jzwiebel@stanford.edu
  Claire Huang | chuang20@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu
  Ben Newman | blnewman@stanford.edu
  Keegan Mehall | kmehall@stanford.edu
  Jesus Cervantes | cerjesus@stanford.edu
  Matthew Tan | mratan@stanford.edu

  File: main.cpp
  --------------------------
  Flight code for main VALBAL avionics.
*/
extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
  int _write(){return -1;}
}


#include "Avionics.h"

/***********************************  BOOT  ***********************************/
Avionics VALBAL;
int main(void) {
  VALBAL.init();
  //VALBAL.test();
/***********************************  MAIN  ***********************************/
  while(true) {
		//Serial.println("loo6aap");
		//delay(1000);
    VALBAL.updateState();
    VALBAL.evaluateState();
    VALBAL.actuateState();
    VALBAL.logState();
    VALBAL.sendComms();
    VALBAL.sleep();
  }
  return 0;
}

/*********************************  CALLBACK  *********************************/
bool ISBDCallback() {
  if(VALBAL.finishedSetup()) {
    VALBAL.updateState();
    VALBAL.evaluateState();
    VALBAL.actuateState();
    VALBAL.logState();
    VALBAL.sleep();
  }
  return true;
}
