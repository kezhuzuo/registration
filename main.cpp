
#include"function.h"

int main(int argc, char** argv)
{
	const int ICP_ITERATIONS = 500; //Number of  icp iterations
	const double SAMPLING_RADIUS = 2.0; //Sampling size, please adjust according to point cloud size and density
	//pcl::pcl::PointCloud<pcl::PointXYZ><pcl::PointXYZ>::Ptr
	pcl::PointCloud<pcl::PointXYZ>::Ptr data_raw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\Desktop\\3\\1.pcd", *data_raw);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_raw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("C:\\Users\\Administrator\\Desktop\\3\\2.pcd", *model_raw);

	clock_t start = clock();
	//Preprocessing£ºinclude downSample£¬normal£¬FPFH
	removeNAN(data_raw);
	double data_radius = SAMPLING_RADIUS;
	auto data = downSample(data_raw, data_radius);
	auto data_normals = pclNormal(data, 2 * data_radius);
	auto fpfhs_data = fpfh(data, data_normals, 4 * data_radius);
	//auto vfh_data = vfh(data, data_normals, 4 * data_radius);

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
	clock_t sac_time = clock();
	std::cout << "SAC-IA " << (scia.hasConverged() ? "converged" : "not converged") << ", score: " << scia.getFitnessScore()
		<< ", time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << std::endl;

	//ICP first time
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(data);
	icp.setInputTarget(model_raw);
	icp.setMaxCorrespondenceDistance(data_radius);
	icp.setMaximumIterations(ICP_ITERATIONS);
	icp.align(*icp_result, scia.getFinalTransformation());
	clock_t icp_time = clock();
	std::cout << "ICP " << (icp.hasConverged() ? "converged" : "not converged")
		<< ", score: " << icp.getFitnessScore()
		<< ", time: " << (double)(icp_time - sac_time) / (double)CLOCKS_PER_SEC << " s\n";

	//ICP second time by using higher density point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	auto data2 = downSample(data_raw, data_radius / M_PI);
	icp2.setInputSource(data2);
	icp2.setInputTarget(model_raw);
	icp2.setMaxCorrespondenceDistance(data_radius / M_PI);
	icp2.setMaximumIterations(ICP_ITERATIONS);
	icp2.align(*icp_result, icp.getFinalTransformation());

	clock_t icp2_time = clock();
	std::cout << "ICP2 " << (icp2.hasConverged() ? "converged" : "not converged") << ", score: " << icp2.getFitnessScore()
		<< ", time: " << (double)(icp2_time - icp_time) / (double)CLOCKS_PER_SEC << std::endl;

	Eigen::Matrix4f icp_trans = icp2.getFinalTransformation();
	std::cout << "Total time: " << (double)(icp2_time - start) / (double)CLOCKS_PER_SEC << std::endl;
	std::cout << "result of registration: \n" << icp_trans << std::endl;

	//save result
	std::ofstream file("transformation.txt");
	file << icp_trans << std::endl;
	std::cout << "Transformation matrix saved!" << std::endl;

	//Save registered Point Cloud
	pcl::io::savePCDFileASCII("C:\\Users\\Administrator\\Desktop\\3\\model_aligned2.pcd", *icp_result);
	std::cout << "Aligned pcl::PointCloud<pcl::PointXYZ> saved!" << std::endl;

	getchar();
	return 0;
}