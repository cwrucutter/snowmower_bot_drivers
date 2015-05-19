/* atan2 example */
#include <stdio.h>      /* printf */
#include <math.h>       /* atan2 */
#include "GPS_Equations.h"

#define PI 3.14159265

int main (int argc, char *argv[])
{

printf("%s",argv[0]);

  sensor_msgs::NavSatFix originGPS;
  sensor_msgs::NavSatFix yPtGPS;
sensor_msgs::NavSatFix rPtGPS;

originGPS.latitude = atof(argv[1]);
originGPS.longitude = atof(argv[2]);
originGPS.altitude = atof(argv[3]);

yPtGPS.latitude = atof(argv[4]);
yPtGPS.longitude = atof(argv[5]);
yPtGPS.altitude = atof(argv[6]);

rPtGPS.latitude = atof(argv[7]);
rPtGPS.longitude = atof(argv[8]);
rPtGPS.altitude = atof(argv[9]);

  geometry_msgs::Point originENU;
  geometry_msgs::Point yPtENU;
geometry_msgs::Point rPtENU;

  GPS_Equations::LLA2ENU(originGPS,originENU);
  GPS_Equations::LLA2ENU(yPtGPS,yPtENU);
GPS_Equations::LLA2ENU(rPtGPS,rPtENU);


printf("\n");
// print LLA
printf("originGPS.latitude = %f, originGPS.longitude = %f\n", originGPS.latitude, originGPS.longitude);
printf("yPtGPS.latitude = %f, yPtGPS.longitude = %f\n",yPtGPS.latitude, yPtGPS.longitude);
// print ENU
printf("originENU.x = %f, originENU.y = %f\n", originENU.x, originENU.y);
printf("yPtENU.x = %f, yPtENU.y = %f\n",yPtENU.x, yPtENU.y);

  double x, y, a;

  x = yPtENU.x-originENU.x;
  y = yPtENU.y-originENU.y;

  a = atan2 (x,y);

printf("x = %f, y = %f, a = %f\n", x,y,a*180/PI);

geometry_msgs::Point rPtMap;

  rPtMap.x = (rPtENU.x-originENU.x)*cos(a)-(rPtENU.y-originENU.y)*sin(a);
  rPtMap.y = (rPtENU.x-originENU.x)*sin(a)+(rPtENU.y-originENU.y)*cos(a);

printf("rTENU.x = %f, rTENU.y = %f\n",rPtENU.x-originENU.x,rPtENU.y-originENU.y);

printf("xcosa = %f, xsina = %f, ycosa = %f, ysina = %f\n",  
	 (rPtENU.x-originENU.x)*cos(a),
	 (rPtENU.x-originENU.x)*sin(a),
	 (rPtENU.y-originENU.y)*cos(a),
	 (rPtENU.y-originENU.y)*sin(a));

printf("rPtMap.x = %f, rPtMap.y = %f\n", rPtMap.x, rPtMap.y);

  return 0;
}
