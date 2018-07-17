#include <stdio.h>
#include "spa.h"  //include the SPA header file

int main (int argc, char *argv[])
{
    SunsetPredictor::GPSTime gtime;
    gtime.year = 2018;
    gtime.month = 0;
    gtime.day = 0;
    gtime.hour = 0;
    gtime.minute = 0;
    gtime.second = 0;
    SunsetPredictor sunpred;
    float lon = -122.1697;
    float lat = 37.4275;
    sunpred.calcValues(lon, lat, )
}