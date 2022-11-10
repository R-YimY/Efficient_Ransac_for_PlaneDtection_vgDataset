// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp
#include<iostream>
#include"../Efficient-RANSAC/OBB_box/PointManager.h"

int main()
{
	std::string fileinput = "F:/POLYFIT-MVS/dataset/output3Cloud3.xyz";
	std::string fileoutput = "F:/POLYFIT-MVS/dataset/efficientoutput-image.vg";
	PointManger pm;
	pm.PointReader(fileinput);
	pm.SetParameter(0.005, 0.02, 0.8, 1000, 0.001);
	pm.ShapeDtector();
	pm.PointExportor(fileoutput);
	system("pause");
	return 0;				
}