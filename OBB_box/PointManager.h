#pragma once
#include <iostream>
#include<fstream>
#include "ctime"
#include <pcl/point_types.h>
#include <PointCloud.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include<basic.h>
#include<string.h>

//平面检测
#include<Plane.h>
#include<RansacShapeDetector.h>
#include<PlanePrimitiveShapeConstructor.h>
#include<PlanePrimitiveShape.h>

#define N  9999 //精度为小数点后面2位

class PointManger {
public:
	void PointReader(std::string filepath);
	void SetParameter(double epsilon, double bitmapEpsilon, double normalThresh,
		double minSupport, double probability);
	void ShapeDtector();
	void PointExportor(std::string filepath);

private:	
	void Compute_obb_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
private:
	PointCloud pc;
	Vec3f boxmin;
	Vec3f boxmax;

	double m_epsilon;
	double m_bitmapEpsilon;
	double m_normalThresh;
	double m_minSupport;
	double m_probability;

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes;
};

//最好是在检测后把多余的点进行删除，这样有利于减少导出后的数据量