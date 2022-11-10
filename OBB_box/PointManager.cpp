#include"PointManager.h"

void PointManger::PointReader(std::string filepath)
{	
	std::ifstream input_file;
	input_file.open(filepath);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10;
	if (!input_file)
	{
		std::cout << "数据导入失败" << std::endl;
		system("pause");
	}
	else
	{
		while (input_file >> d1 >> d2 >> d3 >> d4 >> d5 >> d6 >> d7 >> d8 >> d9 >> d10)
		{
			pc.push_back(Point(Vec3f(d1, d2, d3), Vec3f(d7, d8, d9), Vec3f(d4, d5, d6), d10)); //顺序为xyz norma rgb image
			pcl::PointXYZ pt;
			pt.x = d1;
			pt.y = d2;
			pt.z = d3;
			cloud->push_back(pt);
		}
		std::cout << "数据导入成功： " << pc.size() << std::endl;
	}
	//计算包围盒
	Compute_obb_box(cloud);
}

void PointManger::Compute_obb_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (cloud->size()==0)
	{
		std::cout << "包围盒数据输入无效" << std::endl;
	}
	else {
		std::cout << "##############点云数量：" << cloud->size() << std::endl;
		std::cout << "##############包围盒计算中......" << std::endl;

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);

		std::cout << "min_point_AABB：" << min_point_AABB.x << " " << min_point_AABB.y << " " << min_point_AABB.z << std::endl;
		std::cout << "max_point_AABB：" << max_point_AABB.x << " " << max_point_AABB.y << " " << max_point_AABB.z << std::endl;

		boxmin[0] = min_point_AABB.x;
		boxmin[1] = min_point_AABB.y;
		boxmin[2] = min_point_AABB.z;
		boxmax[0] = max_point_AABB.x;
		boxmax[1] = max_point_AABB.y;
		boxmax[2] = max_point_AABB.z;
	}
}


void PointManger::SetParameter(double epsilon, double bitmapEpsilon, double normalThresh, double minSupport, double probability)
{
	m_epsilon = epsilon;
	m_bitmapEpsilon = bitmapEpsilon;
	m_normalThresh = normalThresh;
	m_minSupport = minSupport;
	m_probability = probability;
}

void PointManger::ShapeDtector()
{
	if (pc.size() != 0) {
		//设置检测参数
		pc.setBBox(boxmin, boxmax);
		RansacShapeDetector::Options ransacOptions;
		ransacOptions.m_epsilon = m_epsilon * pc.getScale();
		ransacOptions.m_bitmapEpsilon = m_bitmapEpsilon * pc.getScale();
		ransacOptions.m_normalThresh = m_normalThresh;
		ransacOptions.m_minSupport = m_minSupport;
		ransacOptions.m_probability = m_probability;
		//设置检测形状（平面检测）
		RansacShapeDetector detector(ransacOptions);
		detector.Add(new PlanePrimitiveShapeConstructor());
		//形状检测及保存
		std::cout << "##############形状检测中......" << std::endl;
		size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);
		std::cout << "##############形状检测完成，共计 " <<shapes.size()<<" 个形状"<< std::endl;
		std::cout << "##############无效点数量：" << remaining << std::endl;
	}
	else {
		std::cout << "数据缺少，无法检测形状！" << std::endl;
	}
}

void PointManger::PointExportor(std::string filepath)
{
	//输出总体点云数据
	std::cout << "##############数据导出中......" << std::endl;
	std::ofstream file(filepath);
	//导出坐标信息
	file << "num_points: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].pos[0] << " " << pc[i].pos[1] << " " << pc[i].pos[2] << " ";
	}
	file << std::endl;

	//导出色彩信息(不保存颜色)
	file << "num_colors: "<<0<<std::endl;

	//导出法线信息
	file << "num_normals: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].normal[0] << " " << pc[i].normal[1] << " " << pc[i].normal[2] << " ";
	}
	file << std::endl;

	//导出影像信息
	file << "num_images: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].image << " ";
	}
	file << std::endl;

	//输出每个形状的参数
	file << "num_groups: " << shapes.size() << std::endl;
	static int end = pc.size();
	double r, g, b;
	srand(time(NULL));
	for (int i = 0; i < shapes.size(); i++)
	{
		r = rand() % (N + 1) / (float)(N + 1);
		g = rand() % (N + 1) / (float)(N + 1);
		b = rand() % (N + 1) / (float)(N + 1);
		PrimitiveShape *primitive = shapes[i].first;
		const Plane &plane = dynamic_cast<PlanePrimitiveShape*>(primitive)->Internal();
		double dx = plane.getNormal()[0] * plane.getPosition()[0] + plane.getNormal()[1] * plane.getPosition()[1] + plane.getNormal()[2] * plane.getPosition()[2];
		
		//平面参数
		file << "group_type: 0" <<std::endl;
		file << "num_group_parameters: " << 4 << std::endl;
		file << "group_parameters: " << plane.getNormal()[0] << " " << plane.getNormal()[1] << " " << plane.getNormal()[2] << " " << -dx << std::endl;
		file << "group_label: unknown" << std::endl;
		file << "group_color: " << r << " " << g << " " << b << std::endl;
		file << "group_num_point: " << shapes[i].second << std::endl;
		
		for (int j = 0; j < shapes[i].second; j++)
		{
			file << j + (end - shapes[i].second) << " ";
		}
		file << std::endl;
		file << "num_children: 0" << std::endl;
		std::cout << "形状 " << i << " 范围：" << end - shapes[i].second << " - " << end << std::endl;
		end = end - shapes[i].second - 1;
	}
	std::cout << "##############文件导出成功! " << std::endl;
}


