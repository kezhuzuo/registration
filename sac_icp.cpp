
#include"function.h"

int main(int argc, char** argv)
{
	/*Eigen::Matrix4d rt;
	rt << 0.940204, -0.264732, 0.214322, -408.038,
		-0.000203461, 0.628787, 0.777577, 213.304,
		-0.340613, -0.731125, 0.591134, 266.783,
		0, 0, 0, 1;
	getAnotherPointCloudFromOne("1.txt", "2.txt", rt);*/

	/*Eigen::Matrix4d rt;
	rt << 0.940312, -0.264631, 0.214013, -408.067,
		-1.08952e-05, 0.628804, 0.777572, 213.304,
		-0.340342, -0.731157, 0.591268, 266.8,
		0, 0, 0, 1;
	getAnotherPointCloudFromOne("1.txt", "3.txt", rt);
	double err = 0;
	calculateError("3.txt", "4.txt", err);
	std::cout << "err= " << err << std::endl;*/

	const int ICP_ITERATIONS = 50; //Number of  icp iterations
	const double SAMPLING_RADIUS = 2.0; //Sampling size, please adjust according to point cloud size and density

	PointCloud::Ptr data_raw(new PointCloud);
	pcl::io::loadPCDFile("data.pcd", *data_raw);
	PointCloud::Ptr model_raw(new PointCloud);
	pcl::io::loadPCDFile("model.pcd", *model_raw);

	//Apply a random pose transformation to the original point cloud
	//Eigen::Isometry3f rand_trans = Eigen::Isometry3f::Identity();
	//std::uniform_real_distribution<float> unif(-M_PI, M_PI);
	//float rand_angle = unif(std::default_random_engine());
	//rand_trans.rotate(Eigen::AngleAxisf(rand_angle, Eigen::Vector3f::Random().normalized()));
	//rand_trans.pretranslate(500 * Eigen::Vector3f::Random());
	//cout << "rand_trans: \n" << rand_trans.matrix() << endl;
	//pcl::transformPointCloud(*data_raw, *data_raw, rand_trans);

	clock_t start = clock();
	//Preprocessing£ºinclude downSample£¬normal£¬FPFH
	removeNAN(data_raw);
	double data_radius = SAMPLING_RADIUS;
	auto data = downSample(data_raw, data_radius);
	auto data_normals = normal(data, 2 * data_radius);
	auto fpfhs_data = fpfh(data, data_normals, 4 * data_radius);

	removeNAN(model_raw);
	double model_radius = SAMPLING_RADIUS / (M_PI / 2.0);
	auto model = downSample(model_raw, model_radius);
	auto model_normals = normal(model, 2 * model_radius);
	auto fpfhs_model = fpfh(model, model_normals, 4 * model_radius);

	// registration of SAC
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(data);
	scia.setInputTarget(model);
	scia.setSourceFeatures(fpfhs_data);
	scia.setTargetFeatures(fpfhs_model);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	pcl::transformPointCloud(*data_raw, *sac_result, scia.getFinalTransformation());
	clock_t sac_time = clock();
	cout << "SAC " << (scia.hasConverged() ? "converged" : "not converged")
		<< ", score: " << scia.getFitnessScore()
		<< ", time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s\n";

	//ICP first time
	PointCloud::Ptr icp_result(new PointCloud);
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
	PointCloud::Ptr icp_result2(new PointCloud);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	auto data2 = downSample(data_raw, data_radius / M_PI);
	icp2.setInputSource(data2);
	icp2.setInputTarget(model_raw);
	icp2.setMaxCorrespondenceDistance(data_radius / M_PI);
	icp2.setMaximumIterations(ICP_ITERATIONS);
	icp2.align(*icp_result, icp.getFinalTransformation());

	clock_t icp2_time = clock();
	std::cout << "ICP2 " << (icp2.hasConverged() ? "converged" : "not converged")<< ", score: " << icp2.getFitnessScore()
		<< ", time: " << (double)(icp2_time - icp_time) / (double)CLOCKS_PER_SEC << std::endl;

	Eigen::Matrix4f icp_trans = icp2.getFinalTransformation();
	std::cout << "Total time: " << (double)(icp2_time - start) / (double)CLOCKS_PER_SEC << std::endl;
	std::cout << "result of registration: \n" << icp_trans << std::endl;

	//save result
	std::ofstream file("transformation.txt");
	file << icp_trans << endl;
	std::cout << "Transformation matrix saved!" << std::endl;

	//Save registered Point Cloud
	pcl::io::savePCDFileASCII("model_aligned.pcd", *icp_result);
	std::cout << "Aligned pointcloud saved!" << std::endl;

	////point cloud Visualization
	//pcl::transformPointCloud(*data_raw, *icp_result, icp_trans);
	//visualize_pcd(data_raw, model_raw, sac_result, icp_result);

	return 0;
}