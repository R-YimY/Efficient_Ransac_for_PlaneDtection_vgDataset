#include"PointManager.h"

void PointManger::PointReader(std::string filepath)
{	
	std::ifstream input_file;
	input_file.open(filepath);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10;
	if (!input_file)
	{
		std::cout << "���ݵ���ʧ��" << std::endl;
		system("pause");
	}
	else
	{
		while (input_file >> d1 >> d2 >> d3 >> d4 >> d5 >> d6 >> d7 >> d8 >> d9 >> d10)
		{
			pc.push_back(Point(Vec3f(d1, d2, d3), Vec3f(d7, d8, d9), Vec3f(d4, d5, d6), d10)); //˳��Ϊxyz norma rgb image
			pcl::PointXYZ pt;
			pt.x = d1;
			pt.y = d2;
			pt.z = d3;
			cloud->push_back(pt);
		}
		std::cout << "���ݵ���ɹ��� " << pc.size() << std::endl;
	}
	//�����Χ��
	Compute_obb_box(cloud);
}

void PointManger::Compute_obb_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (cloud->size()==0)
	{
		std::cout << "��Χ������������Ч" << std::endl;
	}
	else {
		std::cout << "##############����������" << cloud->size() << std::endl;
		std::cout << "##############��Χ�м�����......" << std::endl;

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

		std::cout << "min_point_AABB��" << min_point_AABB.x << " " << min_point_AABB.y << " " << min_point_AABB.z << std::endl;
		std::cout << "max_point_AABB��" << max_point_AABB.x << " " << max_point_AABB.y << " " << max_point_AABB.z << std::endl;

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
		//���ü�����
		pc.setBBox(boxmin, boxmax);
		RansacShapeDetector::Options ransacOptions;
		ransacOptions.m_epsilon = m_epsilon * pc.getScale();
		ransacOptions.m_bitmapEpsilon = m_bitmapEpsilon * pc.getScale();
		ransacOptions.m_normalThresh = m_normalThresh;
		ransacOptions.m_minSupport = m_minSupport;
		ransacOptions.m_probability = m_probability;
		//���ü����״��ƽ���⣩
		RansacShapeDetector detector(ransacOptions);
		detector.Add(new PlanePrimitiveShapeConstructor());
		//��״��⼰����
		std::cout << "##############��״�����......" << std::endl;
		size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);
		std::cout << "##############��״�����ɣ����� " <<shapes.size()<<" ����״"<< std::endl;
		std::cout << "##############��Ч��������" << remaining << std::endl;
	}
	else {
		std::cout << "����ȱ�٣��޷������״��" << std::endl;
	}
}

void PointManger::PointExportor(std::string filepath)
{
	//��������������
	std::cout << "##############���ݵ�����......" << std::endl;
	std::ofstream file(filepath);
	//����������Ϣ
	file << "num_points: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].pos[0] << " " << pc[i].pos[1] << " " << pc[i].pos[2] << " ";
	}
	file << std::endl;

	//����ɫ����Ϣ(��������ɫ)
	file << "num_colors: "<<0<<std::endl;

	//����������Ϣ
	file << "num_normals: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].normal[0] << " " << pc[i].normal[1] << " " << pc[i].normal[2] << " ";
	}
	file << std::endl;

	//����Ӱ����Ϣ
	file << "num_images: "<<pc.size()<<std::endl;
	for (int i(0); i < pc.size(); i++) {
		file << pc[i].image << " ";
	}
	file << std::endl;

	//���ÿ����״�Ĳ���
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
		
		//ƽ�����
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
		std::cout << "��״ " << i << " ��Χ��" << end - shapes[i].second << " - " << end << std::endl;
		end = end - shapes[i].second - 1;
	}
	std::cout << "##############�ļ������ɹ�! " << std::endl;
}


