//
#include"function.h"

int getAnotherPointCloudFromOne(const char* ptr_pcl_file1, const char* ptr_pcl_file2, const Eigen::Matrix4d& rt)
{
	FILE* ptr_file1;
	fopen_s(&ptr_file1, ptr_pcl_file1, "r");
	
	if (NULL == ptr_file1)
	{
		std::cout << "cannot open first file!" << std::endl;
		return -1;
	}

	// load x y z data
	double x = 0.0, y = 0.0, z = 0.0;
	int status = 0;
	int count = 0;
	MyPoint3d pt1;
	std::ofstream file(ptr_pcl_file2);
	while (count<100000)
	{
		//txt1
		status = fscanf_s(ptr_file1, "%lf", &x);
		if (EOF == status)
			break;
		pt1.x = x;

		status = fscanf_s(ptr_file1, "%lf", &y);
		if (EOF == status)
			break;
		pt1.y = y;

		status = fscanf_s(ptr_file1, "%lf", &z);
		if (EOF == status)
			break;
		pt1.z = z;

		Eigen::Vector4d matrix_pt1, matrix_pt2;
		matrix_pt2 << pt1.x, pt1.y, pt1.z, 1;
		matrix_pt1 = rt * matrix_pt2;
		file << matrix_pt1(0, 0) << " " << matrix_pt1(1, 0) << " " << matrix_pt1(2, 0) << std::endl;
		++count;
	}
	return 0;
}


int calculateError(const char* ptr_pcl_file1, const char* ptr_pcl_file2, double& error)
{
	FILE* ptr_file1;
	fopen_s(&ptr_file1, ptr_pcl_file1, "r");

	FILE* ptr_file2;
	fopen_s(&ptr_file2, ptr_pcl_file2, "r");

	if (NULL == ptr_file1)
	{
		std::cout << "cannot open first file!" << std::endl;
		return -1;
	}

	if (NULL == ptr_file2)
	{
		std::cout << "cannot open second file!" << std::endl;
		return -1;
	}

	// load x y z data
	double x = 0.0, y = 0.0, z = 0.0;
	int status = 0;
	int count = 0;
	double sum = 0;
	MyPoint3d pt1, pt2;
	while (1)
	{
		//txt1
		status = fscanf_s(ptr_file1, "%lf", &x);
		if (EOF == status)
			break;
		pt1.x = x;

		status = fscanf_s(ptr_file1, "%lf", &y);
		if (EOF == status)
			break;
		pt1.y = y;

		status = fscanf_s(ptr_file1, "%lf", &z);
		if (EOF == status)
			break;
		pt1.z = z;

		//txt2
		status = fscanf_s(ptr_file2, "%lf", &x);
		if (EOF == status)
			break;
		pt2.x = x;

		status = fscanf_s(ptr_file2, "%lf", &y);
		if (EOF == status)
			break;
		pt2.y = y;

		status = fscanf_s(ptr_file2, "%lf", &z);
		if (EOF == status)
			break;
		pt2.z = z;

		double xi = pt1.x - pt2.x;
		double yi = pt1.y - pt2.y;
		double zi = pt1.z - pt2.z;
		double er = pow(xi, 2) + pow(yi, 2) + pow(zi, 2);
		sum += sqrt(er);
		++count;
	}
	error = sum / count;
	return 0;
}


//calculate numble of point
int calculateNumble(const char* ptr_pcl_file1, int& num)
{
	FILE* ptr_file1;
	fopen_s(&ptr_file1, ptr_pcl_file1, "r");

	if (NULL == ptr_file1)
	{
		std::cout << "cannot open first file!" << std::endl;
		return -1;
	}

	// load x y z data
	double x = 0.0, y = 0.0, z = 0.0;
	int status = 0;
	num = 0;

	MyPoint3d pt1;
	while (1)
	{
		//txt1
		status = fscanf_s(ptr_file1, "%lf", &x);
		if (EOF == status)
			break;
		pt1.x = x;

		status = fscanf_s(ptr_file1, "%lf", &y);
		if (EOF == status)
			break;
		pt1.y = y;

		status = fscanf_s(ptr_file1, "%lf", &z);
		if (EOF == status)
			break;
		pt1.z = z;
		++num;
	}
	return 0;
}

//calculate error(bu internet)
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1) 
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}
	result_angle << ax, ay, az;
}



//reduce NAN
void removeNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) 
{
	std::vector<int> indices_data; //index of NAN
	pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*pc, *pc, indices_data);
}

//filter of downSample 
pcl::PointCloud<pcl::PointXYZ>::Ptr downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, double gridSize) 
{
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(gridSize, gridSize, gridSize);
	voxel_grid.setInputCloud(pc);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*pc_downsampled);
	printf("downsampled to %.2f %%, #points from %d to %d \n",
		100.0 * pc_downsampled->size() / pc->size(),
		pc->size(), pc_downsampled->size());
	return pc_downsampled;
}

//calculate normal of surface
pcl::PointCloud<pcl::Normal>::Ptr pclNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, double radius) 
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_data;
	ne_data.setInputCloud(pc);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_data(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_data.setSearchMethod(tree_data);
	pcl::PointCloud<pcl::Normal>::Ptr pc_normals(new pcl::PointCloud<pcl::Normal>);
	ne_data.setRadiusSearch(radius);
	ne_data.compute(*pc_normals);
	return pc_normals;
}

//calculate FPFH
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, double radius)
{
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(pc);
	fpfh.setInputNormals(pc_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_data_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh.setSearchMethod(tree_data_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.setRadiusSearch(radius);
	fpfh.compute(*fpfhs);
	return fpfhs;
}


//result
void uaiRegisration(const char* file1, const char* file2, float T[4][3], const int ICP_ITERATIONS, const double SAMPLING_RADIUS)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr data_raw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("data.pcd", *data_raw);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_raw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("model.pcd", *model_raw);

	//Preprocessing£ºinclude downSample£¬normal£¬FPFH
	removeNAN(data_raw);
	double data_radius = SAMPLING_RADIUS;
	auto data = downSample(data_raw, data_radius);
	auto data_normals = pclNormal(data, 2 * data_radius);
	auto fpfhs_data = fpfh(data, data_normals, 4 * data_radius);

	removeNAN(model_raw);
	double model_radius = SAMPLING_RADIUS / (M_PI / 2.0);
	auto model = downSample(model_raw, model_radius);
	auto model_normals = pclNormal(model, 2 * model_radius);
	auto fpfhs_model = fpfh(model, model_normals, 4 * model_radius);

	// registration of SAC-IA
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(data);
	scia.setInputTarget(model);
	scia.setSourceFeatures(fpfhs_data);
	scia.setTargetFeatures(fpfhs_model);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZ>);
	scia.align(*sac_result);
	pcl::transformPointCloud<pcl::PointXYZ>(*data_raw, *sac_result, scia.getFinalTransformation());

	//ICP first time
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(data);
	icp.setInputTarget(model_raw);
	icp.setMaxCorrespondenceDistance(data_radius);
	icp.setMaximumIterations(ICP_ITERATIONS);
	icp.align(*icp_result, scia.getFinalTransformation());

	//ICP second time by using higher density point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	auto data2 = downSample(data_raw, data_radius / M_PI);
	icp2.setInputSource(data2);
	icp2.setInputTarget(model_raw);
	icp2.setMaxCorrespondenceDistance(data_radius / M_PI);
	icp2.setMaximumIterations(ICP_ITERATIONS);
	icp2.align(*icp_result, icp.getFinalTransformation());

	Eigen::Matrix4f icp_trans = icp2.getFinalTransformation();
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			T[i][j] = icp_trans(i, j);
		}
	}

	T[3][0] = icp_trans(0, 3);
	T[3][1] = icp_trans(1, 3);
	T[3][2] = icp_trans(2, 3);
}
