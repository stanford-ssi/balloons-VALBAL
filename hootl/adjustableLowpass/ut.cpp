#include <iostream>
#include <fstream>
#include <iomanip>
#include "Utils.h"

using namespace std;

int main ()
{
  AdjustableLowpass lp(1,.9,20);
  for(int i = 0; i < 100; i++){
  	printf("%f\n", lp.update(1.0));
  }
}

