#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "Utils.h"  //include the SPA header file

using namespace std;

int main (int argc, char *argv[])
{
    fstream o ("output.bin", std::fstream::out | std::fstream::binary);
    SunsetPredictor::GPSTime gtime;

    //at stanford
    float lon = -122.1697;
    float lat = 37.4275;
    gtime.year = 2018;
    gtime.month = 1;
    gtime.day = 1;
    gtime.hour = 21;
    gtime.minute = 0;
    gtime.second = 0;
    SunsetPredictor sunpred;

    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 6;
    gtime.day = 1;
    gtime.hour = 24;
    gtime.minute = 0;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 3;
    gtime.day = 1;
    gtime.hour = 22;
    gtime.minute = 30;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 9;
    gtime.day = 1;
    gtime.hour = 22;
    gtime.minute = 30;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    lat = 80.0;
    lon = -122.1697;
    gtime.year = 2018;
    gtime.month = 1;
    gtime.day = 1;
    gtime.hour = 21;
    gtime.minute = 0;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 6;
    gtime.day = 1;
    gtime.hour = 24;
    gtime.minute = 0;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 3;
    gtime.day = 1;
    gtime.hour = 22;
    gtime.minute = 30;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }

    gtime.year = 2018;
    gtime.month = 9;
    gtime.day = 1;
    gtime.hour = 22;
    gtime.minute = 30;
    gtime.second = 0;
    for(int s = 1; s<60*6*60; s++){
        sunpred.calcValues(lon, lat, gtime, s);
        //printf("%f, %f, %f \n",sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt);
        printf("%f \n",sunpred.spa.jd);
        double buf[3] = {sunpred.solar_elevation,sunpred.dsedt,sunpred.estimated_dldt};
        o.write((char*)buf,sizeof(buf));
    }
}