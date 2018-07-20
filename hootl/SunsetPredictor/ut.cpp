#include <stdio.h>
#include "Utils.h"  //include the SPA header file

int main (int argc, char *argv[])
{

    SunsetPredictor::GPSTime gtime;
    gtime.year = 2018;
    gtime.month = 1;
    gtime.day = 1;
    gtime.hour = 1;
    gtime.minute = 0;
    gtime.second = 0;
    SunsetPredictor sunpred;
    float lon = -122.1697;
    float lat = 37.4275;
    sunpred.calcValues(lon, lat, gtime);
    printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
}