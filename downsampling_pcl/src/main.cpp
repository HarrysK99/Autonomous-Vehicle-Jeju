#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <dynamic_reconfigure/server.h>
#include <my_pcl_tutorial/my_pcl_tutorial_Config.h>



#define Point2 pcl::PointXYZI

typedef pcl::PointXYZI pointTypeIO;


using namespace std;

ros::Publisher pub;
ros::Publisher pub_inlier;
ros::Publisher pub_outlier;
ros::Publisher pub_outlier_withROI;
//ros::Publisher pub5;

int MAXITERATIONS;
double MAXDISTANCE_X, MINDISTANCE_X;
double MAXDISTANCE_Y, MINDISTANCE_Y;
double MAXDISTANCE_Z, MINDISTANCE_Z;
double DISTANCETHRESHOLD, BOUNDARY;

void callback(my_pcl_tutorial::my_pcl_tutorial_Config &config, uint32_t level) {
    MAXITERATIONS = config.MaxIterations;
	DISTANCETHRESHOLD = config.DistanceThreshold;
	BOUNDARY = config.Boundary;
	MAXDISTANCE_X = config.MaxDistance_x;
	MINDISTANCE_X = config.MinDistance_x;
	MAXDISTANCE_Y = config.MaxDistance_y;
	MINDISTANCE_Y = config.MinDistance_y;
	MAXDISTANCE_Z = config.MaxDistance_z;
	MINDISTANCE_Z = config.MinDistance_z;
}


void plane_segmentation(const sensor_msgs::PointCloud2::ConstPtr& scan){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
										cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
										center_x(new pcl::PointCloud<pcl::PointXYZ>),
										center_y(new pcl::PointCloud<pcl::PointXYZ>),
										cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>),
										cloud_clustered_temp(new pcl::PointCloud<pcl::PointXYZ>),
										cloud_3(new pcl::PointCloud<pcl::PointXYZ>),
										cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::fromROSMsg(*scan, *cloud);
	pcl::fromROSMsg(*scan,*cloud);

  	std::cout << "Input : " << cloud->points.size () << " (" << pcl::getFieldsList (*cloud) <<")"<< std::endl; 

	//Downsampling========================================
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.02f, 0.02f, 0.02f);
	sor.filter(*cloud_downsampled);
	std::cout<<"Output: "<<cloud_downsampled->points.size()<<std::endl;
	//=====================================================

	//ROI 설정=============================================
	pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud(cloud_clustered_temp);
	//pass.setInputCloud(cloud);
	pass.setInputCloud(cloud_downsampled);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(MINDISTANCE_X, MAXDISTANCE_X);
	pass.filter(*center_x);

	pass.setInputCloud(center_x);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(MINDISTANCE_Y, MAXDISTANCE_Y);
	pass.filter(*center_y);

	pass.setInputCloud(center_y);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(MINDISTANCE_Z, MAXDISTANCE_Z);
	pass.filter(*cloud_filtered);

	std::cout<<"Filtered: "<<cloud_filtered->width * cloud_filtered->height << std::endl;

	pcl::PCLPointCloud2 cloud_clustered_center;
	pcl::toPCLPointCloud2(*cloud_filtered,cloud_clustered_center);
	sensor_msgs::PointCloud2 output_clustered_center;
	pcl_conversions::fromPCL(cloud_clustered_center,output_clustered_center);
	output_clustered_center.header.frame_id = "zed2i_left_camera_frame";
	// output_clustered_center.header.frame_id = "fsds/FSCar";
	pub_outlier_withROI.publish(output_clustered_center);
	//=====================================================

	//Plane Segmentation===================================
	/*
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
	seg.setInputCloud (cloud);                 //입력 
	seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
	seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
	// seg.setMaxIterations (MAXITERATIONS);               //최대 실행 수
	seg.setDistanceThreshold (DISTANCETHRESHOLD);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
	//seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
	seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_clustered_temp);

	pcl::ExtractIndices<pcl::PointXYZ> ec;
	ec.setInputCloud(cloud);
	ec.setIndices(inliers);
	ec.setNegative(true);
	ec.filter(*cloud_3);

	std::cout << "Filtered1 :" << cloud_clustered_temp->width * cloud_clustered_temp->height  << std::endl;  
	std::cout << "Filtered2 :" << cloud_3->width * cloud_3->height  << std::endl;  

	pcl::PCLPointCloud2 cloud_clustered, cloud_clustered2;
	pcl::toPCLPointCloud2(*cloud_clustered_temp, cloud_clustered);
	pcl::toPCLPointCloud2(*cloud_3, cloud_clustered2);
	
	sensor_msgs::PointCloud2 output_clustered, output_clustered2;
	pcl_conversions::fromPCL(cloud_clustered, output_clustered);
	pcl_conversions::fromPCL(cloud_clustered2, output_clustered2);
	output_clustered.header.frame_id = "zed2i_left_camera_frame";
	output_clustered2.header.frame_id = "zed2i_left_camera_frame";
	// output_clustered.header.frame_id = "fsds/FSCar";
	// output_clustered2.header.frame_id = "fsds/FSCar";
	pub_inlier.publish(output_clustered);
	pub_outlier.publish(output_clustered2);
	*/
	//=====================================================

}

int main(int argc, char **argv){
	ros::init(argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server <my_pcl_tutorial::my_pcl_tutorial_Config> server;
	dynamic_reconfigure::Server <my_pcl_tutorial::my_pcl_tutorial_Config>::CallbackType f;

	f=boost::bind(&callback, _1, _2);
	server.setCallback(f);

	// ros::Subscriber sub = nh.subscribe("os_cloud_node/points",1,cloud_cb);
	// ros::Subscriber sub= nh.subscribe("os_cloud_node/points",1,input);
	ros::Subscriber sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered",1,plane_segmentation);
	// ros::Subscriber sub = nh.subscribe("fsds/lidar/Lidar1",1,plane_segmentation);
	pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);
	pub_inlier = nh.advertise<sensor_msgs::PointCloud2>("inlier_points",1);
	pub_outlier = nh.advertise<sensor_msgs::PointCloud2>("outlier_points",1);
	pub_outlier_withROI = nh.advertise<sensor_msgs::PointCloud2>("inlier_points_withROI",1);
	//pub5 = nh.advertise<sensor_msgs::PointCloud2>("final_clustered",1);

	ros::spin();

	return 0;
}