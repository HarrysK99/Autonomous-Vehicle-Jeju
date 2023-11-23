
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>

#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "autoware_msgs/CloudClusterArray.h"
#include <autoware_msgs/DetectedObjectArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/visualization/cloud_viewer.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>

#include <autoware_msgs/DetectedObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <dbscan_segment_origin/dbscan_segment_origin_Config.h>
// #include "obstacle_detector.hpp"
#include "dbscan_kdtree.hpp"

#include "cluster.h"
// #include "common_defs.h"

#include <eigen3/Eigen/Dense>


#define NOT_DETECTED -1

using namespace cv;

typedef pcl::PointXYZ PointType;

std::vector<cv::Scalar> _colors;
static bool _pose_estimation=false;
static const double _initial_quat_w = 1.0;

std::string output_frame="fsds/FSCar";
std_msgs::Header _velodyne_header;

// Pointcloud Filtering Parameters
bool USE_PCA_BOX;
bool USE_TRACKING;
float VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
float CLUSTER_THRESH, ClusterTolerance;
int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE, CorePointMinPt, MinClusterSize, MaxClusterSize;

// void vis_cones_loc(std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubCones);
// void visTrajectory(std::vector<double> coefs, double interval, double ROILength, int idx, std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher pubPolyMarkerArray);
// std::vector<double> curveFitting3(std::vector<std::vector<double>> conePos);
// std::vector<std::vector<double>> sortCones(std::vector<LIDAR::ConeDescriptor> &cones, int axis, int num);
// void generate_path(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud_ptr, std::vector<pcl::PointIndices>& object_indices, ros::Publisher& pubPolyMarkerArray);


class cloud_segmentation
{
 private:

  std::shared_ptr<DBSCAN_KDTREE<PointType>> dbscan_kdtree;
  // std::shared_ptr<LShapedFIT> L_shape_Fit; 

  ros::NodeHandle nh;
  // tf2_ros::Buffer tf2_buffer;
  // tf2_ros::TransformListener tf2_listener;
  tf::TransformListener *_transform_listener;
  tf::StampedTransform *_transform;

  dynamic_reconfigure::Server<dbscan_segment_origin::dbscan_segment_origin_Config> server;
  dynamic_reconfigure::Server<dbscan_segment_origin::dbscan_segment_origin_Config>::CallbackType f;

  pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloudColor;

  ros::Subscriber sub_lidar_points;
  ros::Publisher pub_cloud_ground;
  ros::Publisher pub_cloud_clusters;
  ros::Publisher pub_jsk_bboxes;
  ros::Publisher pub_autoware_objects;
  ros::Publisher _pub_autoware_clusters_message;
  ros::Publisher _pub_autoware_detected_objects;
  ros::Publisher _pub_roi_area;
  ros::Publisher _pubSegmentedCloudColor;
  ros::Publisher pubPolygonCones;
  ros::Publisher pubCones;
  ros::Publisher pubPolyMarkerArray;
  ros::Publisher pubConesIntensity;

  
  ros::Publisher pubTransformedPolyMarkerArray;
  ros::Publisher pubTransformedPathPoints;
  //트랜스폼 한 걸 퍼블리시하도록 퍼블리셔 만들기 2222

  ros::Subscriber sub_car_odometry;
  // ros::Subscriber sub_car_orientation;
  //오도메트리 서브스크라이버 만들기 2222

  // ros::Subscriber sub_red_cone;
  // ros::Publisher pub_brake_signal;
  // red_cone 3333
  
  void vis_cones_loc(std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubCones);
  void vis_cones_info(std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubConesIntensity);
  void visTrajectory(std::vector<double> coefs, double interval, double ROILength, int idx, std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubPolyMarkerArray);
  std::vector<double> curveFitting3(std::vector<std::vector<double>> conePos);
  std::vector<std::vector<double>> sortCones(std::vector<LIDAR::ConeDescriptor> &cones, int axis, int num);
  void generate_path(const Cloud::Ptr in_cloud_ptr, std::vector<pcl::PointIndices>& object_indices, ros::Publisher& pubPolyMarkerArray);

  void pathPointsTransform(visualization_msgs::MarkerArray& pathPoints, ros::Publisher& pubTransformedPolyMarkerArray, ros::Publisher& pubTransformedPathPoints);
  void OdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  // void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // void orientationCallback(const autonomous_message::autonomous_message::ConstPtr& msg);
  //함수 선언 2222

  //카메라로부터 reCone 인덱스를 받아서 정지 signal을 publish한다.
  // void redconeCallback(const std_msgs::Int32::ConstPtr& msg);
  // 3333

  void lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points);
  //jsk_recognition_msgs::BoundingBox transformJskBbox(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);
  //autoware_msgs::DetectedObject transformAutowareObject(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);
  pcl::PointCloud<PointType>::Ptr roi_filter_pcl(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg, const float filter_res);
  pcl::PointCloud<PointType>::Ptr roi_rectangle_filter(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt);
  //Box pcaBoundingBox(pcl::PointCloud<PointType>::Ptr& cluster, const int id);
  //Box L_shape_BBox(const pcl::PointCloud<PointType>::Ptr& cluster, const int id);
  void publish_polygon_cones(const geometry_msgs::Polygon& polygon_cones);

  void dbscan_kdtree_origin(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud_ptr, 
  autoware_msgs::CloudClusterArray &in_out_clusters, const float CorePointMinPt, const float ClusterTolerance, const float MinClusterSize, const float MaxClusterSize);
  void publishSegmentedCloudsColor(const std_msgs::Header& header);
  void publish_ROI_area(const std_msgs::Header& header);
  void publish_autoware_cloudclusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                        const std::string &in_target_frame, const std_msgs::Header &in_header);

 public:
  cloud_segmentation();
  ~cloud_segmentation() {};

  void allocateMemory(){
    segmentedCloudColor.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  void resetParameters(){
    segmentedCloudColor->clear();
  }

};

int8_t checkFlag = 0;
double carPosX, carPosY, roll, pitch, carPosyaw, delta_x, delta_y;
double previous_loc_x = 0.0, previous_loc_y = 0.0;
void cloud_segmentation::OdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  static tf::TransformBroadcaster br;
  // 객체 생성
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0));
  carPosX = msg->pose.position.x;
  carPosY = msg->pose.position.y;
  /////// 위치 x, y
  delta_x = carPosX - previous_loc_x;
  delta_y = carPosY - previous_loc_y;
  carPosyaw = atan2(delta_y, delta_x);

  previous_loc_x = carPosX;
  previous_loc_y = carPosY;

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, carPosyaw);
  transform.setRotation(q);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "os_sensor"));
  checkFlag=1;
}

//함수 정의 2222

//3333
// void cloud_segmentation::redconeCallback(const std_msgs::Int32::ConstPtr& msg) 
// {
//   if(msg.data == NOT_DETECTED)
    

//   std_msgs::Int8 brake_signal;

//   pub_brake_signal.publish(brake_signal);
// }

void cloud_segmentation::vis_cones_info(std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubConesIntensity){
  std::cout<<"vis_cones_info 실행"<<std::endl;

  visualization_msgs::MarkerArray markerArray;
  int id = 0;

  // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
  for(int idx= 0; idx<cones.size(); idx++){
    // if(cones[idx].radius> 0.4 || abs(cones[idx].mean.y) > 3.0 ){
    //     continue;
    // }
    int vis_cones_idx = idx;
    double x = cones[idx].mean.x;
    double y = cones[idx].mean.y;

    visualization_msgs::Marker marker;
    // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
    // marker.header.frame_id = "map";
    // marker.header.frame_id = "fsds/FSCar";
    marker.header.frame_id = "os_sensor";
    marker.header.stamp = ros::Time::now();

    // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
    marker.ns = "vis_cones";
    marker.id = id;                
    marker.text = "Intensity: "+ std::to_string(cones[idx].mean.intensity) 
                   // + " | Valid: " + (cones[idx].valid ? "true" : "false")
                   + " | Size: " + std::to_string(cones[idx].cloud->size())
                   + " | Distance: " + std::to_string(sqrt(pow(x, 2)+pow(y, 2)))
                   // + " | x: " + std::to_string(x)
                   + " | y: " + std::to_string(y)
                   + "\nRadius: " + std::to_string(cones[idx].radius)
                   + "| Stddev: " + std::to_string(cones[idx].stddev.x) + std::to_string(cones[idx].stddev.y)
                   + "| cone[idx]: " + std::to_string(vis_cones_idx);

                                
    // marker.type = visualization_msgs::Marker::CYLINDER;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.scale.x = 0.13;
    marker.scale.y = 0.13;
    marker.scale.z = 0.13;
    marker.lifetime = ros::Duration(0.1);

    markerArray.markers.push_back(marker);
    id++;
  }
  pubConesIntensity.publish(markerArray);
}


void cloud_segmentation::pathPointsTransform(visualization_msgs::MarkerArray& pathPoints, ros::Publisher& pubTransformedPolyMarkerArray, ros::Publisher& pubTransformedPathPoints)
{
  visualization_msgs::MarkerArray transformedMarkerArray;
  nav_msgs::Path tf_lane;
  geometry_msgs::PoseStamped globalcenter_points;

  double cos_yaw = cos(carPosyaw);
  double sin_yaw = sin(carPosyaw);

  for(int id = 0;id<pathPoints.markers.size();id++)
  {
    visualization_msgs::Marker marker;

    // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
    marker.header.frame_id = "world";
    // marker.header.frame_id = "os_sensor";

    marker.header.stamp = ros::Time::now();

    // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
    marker.ns = "polylane";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

        
    double pathpoints_in_car_x = pathPoints.markers[id].pose.position.x;
    double pathpoints_in_car_y = pathPoints.markers[id].pose.position.y;

    marker.pose.position.x = (cos_yaw * pathpoints_in_car_x) - (sin_yaw * pathpoints_in_car_y) + carPosX;
    marker.pose.position.y = (sin_yaw * pathpoints_in_car_x) + (cos_yaw * pathpoints_in_car_y) + carPosY;
    marker.pose.position.z = carPosyaw;

    //std::cout<<"경로 x:"<<marker.pose.position.x<<std::endl;
    //std::cout<<"경로 y:"<<marker.pose.position.y<<std::endl;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 1.0f;

    marker.color.a = 1.0;
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.lifetime = ros::Duration(5);

    globalcenter_points.pose.position.x = marker.pose.position.x;
    globalcenter_points.pose.position.y = marker.pose.position.y;
    globalcenter_points.pose.position.z = marker.pose.position.z;

    globalcenter_points.pose.orientation.x = marker.pose.orientation.x;
    globalcenter_points.pose.orientation.y = marker.pose.orientation.y;
    globalcenter_points.pose.orientation.z = marker.pose.orientation.z;
    globalcenter_points.pose.orientation.w = marker.pose.orientation.w;

    tf_lane.poses.push_back(globalcenter_points);
    transformedMarkerArray.markers.push_back(marker);
  } 
  pubTransformedPolyMarkerArray.publish(transformedMarkerArray);
  pubTransformedPathPoints.publish(tf_lane);
}

void cloud_segmentation::vis_cones_loc(std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubCones)
{
  std::cout<<"vis_cones_loc 실행"<<std::endl;

  visualization_msgs::MarkerArray markerArray;
  int id = 0;
  float my_color[6][3]={
      1.0, 0.0, 0.0,
      1.0, 1.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
      1.0, 0.0, 1.0,
      0.0, 1.0, 1.0
  };

  for(int idx= 0; idx<cones.size(); idx++)
  {
      double x = cones[idx].mean.x;
      double y = cones[idx].mean.y;

      visualization_msgs::Marker marker;

      marker.header.frame_id = "os_sensor";
      marker.header.stamp = ros::Time::now();

      marker.ns = "vis_cones";
      marker.id = id;

      marker.type = visualization_msgs::Marker::CYLINDER;
      
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = -0.4;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.1;
      marker.pose.orientation.w = 1.0;
      
      if(y>0 ){ //y 양수 -> 왼쪽
          marker.color.r=0.0f;
          marker.color.g=0.0f;
          marker.color.b=1.0f;
      }
      else{
          marker.color.r=1.0f;
          marker.color.g=1.0f;
          marker.color.b=0.0f;
      }

      marker.color.a = 1.0;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.4;
      marker.lifetime = ros::Duration(0.1);

      markerArray.markers.push_back(marker);
      id++;
  }

  pubCones.publish(markerArray);
}

void cloud_segmentation::visTrajectory(std::vector<double> coefs, double interval, double ROILength, int idx, std::vector<LIDAR::ConeDescriptor> &cones, ros::Publisher& pubPolyMarkerArray) 
{
  std::cout<<"visTrajectory 실행"<<std::endl;

  visualization_msgs::MarkerArray markerArray;
  // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
  double a0 = coefs.at(0);
  double a1 = coefs.at(1);
  double a2 = coefs.at(2);
  double a3 = (cones[idx].left_cone_location + cones[idx].right_cone_location) / 2;
  

  double x = -10.0;
  double y = a0;

  double distance_square = x * x + y * y;
  
  int id = 0;
  while (distance_square < ROILength * ROILength) 
  {
      y = a0 + a1 * x;
      // y = a0 + a1 * x + a2 * x * x; //a3 * x * x * x;
      distance_square = x * x + y * y;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "os_sensor";

      marker.header.stamp = ros::Time::now();

      marker.ns = "polylane";
      marker.id = id;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      if(idx == 0){
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = 0.0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.1;
          marker.pose.orientation.w = 1.0;
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
      }
      else if(idx == 1){
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = 0.0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.1;
          marker.pose.orientation.w = 1.0;
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
      }
      else if(idx == 2){
          nav_msgs::Path no_tf_lane;
          geometry_msgs::PoseStamped localcenter_points;
          marker.pose.position.x = x;
          marker.pose.position.y = a3 + a1 * x;

          marker.pose.position.z = 0.0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.1;
          marker.pose.orientation.w = 1.0;
          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 1.0f;
      }
      
      marker.color.a = 1.0;
      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      marker.lifetime = ros::Duration(5);
      markerArray.markers.push_back(marker);
      x += interval;
      id++;
  }
  if (idx == 0)
  {
      pubPolyMarkerArray.publish(markerArray);
  }
  else if(idx == 1)
  {
      pubPolyMarkerArray.publish(markerArray);
  }
  else if(idx == 2)
  {
      std::cout<<markerArray.markers.size()<<std::endl;
      // pubPolyMarkerArray.publish(markerArray);
      //트랜스폼 코드 추가 2222
      if(checkFlag==1){
        // pathPointsTransform(markerArray, pubTransformedPolyMarkerArray,pubTransformedPathPoints);
        checkFlag=0;
      }

  }
            
}

std::vector<std::vector<double>> cloud_segmentation::sortCones(std::vector<LIDAR::ConeDescriptor> &cones, int axis, int num)
{
    std::cout<<"sortCones 실행"<<std::endl;
    /*
    axis = 0: xy distance
    axis = 1: x
    axis = 2: y
    axis = 3: RL_status LEFT
    axis = 4: RL_status RIGHT
    */
    std::vector<std::vector<double>> conesBeforeSort;
    int cnt = 0;
    if (axis == 0)
    { //xy distance
        for(auto cone : cones){
            double x = cone.mean.x;
            double y = cone.mean.y;
            double distance = sqrtf(pow(x, 2) + pow(y, 2));
            std::vector<double> dis_idx_tmp;
            dis_idx_tmp.push_back(distance);
            dis_idx_tmp.push_back(x);
            dis_idx_tmp.push_back(y);
            conesBeforeSort.push_back(dis_idx_tmp);
            cnt++;
            if (cnt > num){
                break;
            }
        }  
    }
    else if(axis == 2)
    { // sort y
        for(auto cone : cones){
            double x = cone.mean.x;
            double y = cone.mean.y;
            // double distance = sqrtf(pow(x, 2) + pow(y, 2));
            std::vector<double> dis_idx_tmp;
            // dis_idx_tmp.push_back(distance);
            dis_idx_tmp.push_back(y);
            dis_idx_tmp.push_back(x);
            conesBeforeSort.push_back(dis_idx_tmp);
            cnt++;
            if (cnt > num){
                break;
            }
        } 
    }

    sort(conesBeforeSort.begin(), conesBeforeSort.end());

    std::vector<std::vector<double>> conesSortResult;
    for (int i = 0; i <num; i++){
        double x = 0.;
        double y = 0.;
        if(axis == 0){
            x = conesBeforeSort[i][1];
            y = conesBeforeSort[i][2];
        }
        else if(axis == 2){
            x = conesBeforeSort[i][1];
            y = conesBeforeSort[i][0];
        }

        std::vector<double> xy_pos;
        xy_pos.push_back(x);
        xy_pos.push_back(y);
        conesSortResult.push_back(xy_pos);
    }
    std::cout<<"Num Of Cones: " << std::to_string(conesSortResult.size()) << std::endl;
    return conesSortResult;
}

std::vector<double> cloud_segmentation::curveFitting3(std::vector<std::vector<double>> conePos)
{
    std::cout<<"curveFitting3 실행"<<std::endl;
    int down_size = (conePos.size());
    Eigen::MatrixXd X_Matrix(down_size, 3);
    Eigen::VectorXd y_Vector(down_size);
    Eigen::VectorXd a_Vector(3);

    // Eigen의 매트릭스에 포인트를 넣어준다.
    for (int i_point = 0; i_point < down_size; i_point++) {
        double x = conePos[i_point][0];
        double y = conePos[i_point][1];

        X_Matrix(i_point, 0) = 1;
        X_Matrix(i_point, 1) = x;
        X_Matrix(i_point, 2) = x * x;
        // X_Matrix(i_point, 3) = x * x * x;
        y_Vector(i_point) = y;
    }

    a_Vector = ((X_Matrix.transpose() * X_Matrix).inverse() * X_Matrix.transpose()) * y_Vector;
    std::vector<double> curveCoefs;
    curveCoefs.push_back(a_Vector(0));
    curveCoefs.push_back(a_Vector(1));
    curveCoefs.push_back(a_Vector(2));
    curveCoefs.push_back(0.0);//a_Vector(3));
    return curveCoefs;
}

void cloud_segmentation::generate_path(const Cloud::Ptr in_cloud_ptr, std::vector<pcl::PointIndices>& object_indices, ros::Publisher& pubPolyMarkerArray)
{
  std::cout<<"1"<<std::endl;

  std::vector<LIDAR::ConeDescriptor> cones;
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud_ptr);
  
  cones.reserve(object_indices.size());
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  pcl::PointIndices::Ptr object_ptr(new pcl::PointIndices);
  for (auto& object: object_indices) {
      size_t cluster_size = object.indices.size();
      if (cluster_size < min_size) {
          min_size = cluster_size;
      }
      if (cluster_size > max_size) {
          max_size = cluster_size;
      }
      LIDAR::ConeDescriptor cone;
      *object_ptr = object;
      extract.setIndices(object_ptr);
      extract.filter(*cone.cloud);
      cone.calculate();
      cones.push_back(cone);
  }

  vis_cones_info(cones, pubConesIntensity);

  int fittingCone = 6;
  if (cones.size() >= fittingCone )
  {
    std::vector<std::vector<double>> sortedCones = sortCones(cones, 2, fittingCone);

    std::vector<std::vector<double>> rCones, lCones;
    int tmp=-1;
    for(int i=0;i<fittingCone;i++){
        if(sortedCones[i][1]>0 ){
            std::cout<<"왼쪽!"<<std::endl;
            lCones.push_back(sortedCones[i]);
        }                
        else if(sortedCones[i][1]<0){
            std::cout<<"오른쪽"<<std::endl;
            rCones.push_back(sortedCones[i]);
        }
        else if(sortedCones[i][1]==0){
            std::cout<<"중앙"<<std::endl;
            tmp=i;
        }
    }
    std::vector<double> rCoefs = curveFitting3(std::vector<std::vector<double>>(rCones.begin(), rCones.end()));
    std::vector<double> lCoefs = curveFitting3(std::vector<std::vector<double>>(lCones.begin(), lCones.end()));

    // 상위 3개와 하위 3개에 대해 콘끼리 중심점 연산 및 Curve Fittitng
    // std::vector<double> rCoefs = curveFitting3(std::vector<std::vector<double>>(sortedCones.begin(), sortedCones.begin()+2));
    // std::vector<double> lCoefs = curveFitting3(std::vector<std::vector<double>>(sortedCones.begin()+3, sortedCones.end()));
    std::vector<double> centerCoefs = lCoefs;
    // std::vector<double> rCoefs = curveFitting2(std::vector<std::vector<double>>(sortedCones.begin(), sortedCones.begin()+1));
    // std::vector<double> lCoefs = curveFitting2(std::vector<std::vector<double>>(sortedCones.begin()+2, sortedCones.end()));
    std::cout<<"*******************"<<std::endl;
    // vis Trajectory!!
    visTrajectory(centerCoefs, 0.3, 15, 2, cones, pubPolyMarkerArray);
  }
  vis_cones_info(cones, pubConesIntensity);    
}

// Dynamic parameter server callback function
void dynamicParamCallback(dbscan_segment_origin::dbscan_segment_origin_Config& config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  VOXEL_GRID_SIZE = config.voxel_grid_size;
  CorePointMinPt=config.CorePointMinPt;
  ClusterTolerance=config.ClusterTolerance;
  MinClusterSize=config.MinClusterSize;
  MaxClusterSize=config.MaxClusterSize;
  ROI_MAX_POINT = Eigen::Vector4f(config.roi_max_x, config.roi_max_y, config.roi_max_z, 1);
  ROI_MIN_POINT = Eigen::Vector4f(config.roi_min_x, config.roi_min_y, config.roi_min_z, 1);
}

// GroundPlaneFit::GroundPlaneFit():node_handle_("~"){
//  : tf2_listener(tf2_buffer)
cloud_segmentation::cloud_segmentation(){

  ros::NodeHandle private_nh("~");
  allocateMemory();

  std::string lidar_points_topic_ground;
  std::string lidar_points_topic;
  std::string cloud_ground_topic;
  std::string cloud_clusters_topic;
  std::string jsk_bboxes_topic;
  //std::string autoware_objects_topic;

  lidar_points_topic="/inlier_points_withROI";
  //ROS_ASSERT(private_nh.getParam("lidar_points_topic", lidar_points_topic));
  // ROS_ASSERT(private_nh.getParam("cloud_ground_topic", cloud_ground_topic));
  //ROS_ASSERT(private_nh.getParam("cloud_clusters_topic", cloud_clusters_topic));
  // ROS_ASSERT(private_nh.getParam("jsk_bboxes_topic", jsk_bboxes_topic));
  //ROS_ASSERT(private_nh.getParam("autoware_objects_topic", autoware_objects_topic));


  sub_lidar_points = nh.subscribe(lidar_points_topic, 1, &cloud_segmentation::lidarPointsCallback, this);
  // pub_cloud_ground = nh.advertise<sensor_msgs::PointCloud2>(cloud_ground_topic, 1);
  // pub_cloud_clusters = nh.advertise<sensor_msgs::PointCloud2>(cloud_clusters_topic, 1);
  // pub_jsk_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic,1);
  // pub_autoware_objects = nh.advertise<autoware_msgs::DetectedObjectArray>(autoware_objects_topic, 1);
  _pubSegmentedCloudColor = nh.advertise<sensor_msgs::PointCloud2> ("/detection/segmented_cloud_color_marker", 1);
  _pub_autoware_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
  _pub_autoware_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
  _pub_roi_area = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/roi_area", 1);  
  pubCones = nh.advertise<visualization_msgs::MarkerArray>("/cones_location", 1);
  pubPolyMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/local_center", 1);
  pubConesIntensity = nh.advertise<visualization_msgs::MarkerArray>("/cones_Intensity", 1);
  // nh.advertise 2222
  // 3333
  // pub_brake_signal = nh.advertise<std_msgs::Int8>("brake_control_signal", 1);
  // sub_red_cone = nh.subscribe("/red_cone", 1, &cloud_segmentation::redconeCallback, this);
  
  // sub_car_odometry = nh.subscribe("/utm", 1, &cloud_segmentation::OdometryCallback, this);
  pubTransformedPolyMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("transformed_lane",1);
  // pubTransformedPathPoints = nh.advertise<nav_msgs::Path>("/transformed_pathpoints", 1);

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Create point processor
  // obstacle_detector = std::make_shared<ObstacleDetector<PointType>>();
  dbscan_kdtree = std::make_shared<DBSCAN_KDTREE<PointType>>();
 
  resetParameters();
}

void cloud_segmentation::publishSegmentedCloudsColor(const std_msgs::Header& header)
{
  std::cout<<"publishSegmentedClousColor"<<std::endl;
  sensor_msgs::PointCloud2 segmentedCloudColor_ros;
  
  // extract segmented cloud for visualization
  if (_pubSegmentedCloudColor.getNumSubscribers() != 0){
    pcl::toROSMsg(*segmentedCloudColor, segmentedCloudColor_ros);
    segmentedCloudColor_ros.header = header;
    _pubSegmentedCloudColor.publish(segmentedCloudColor_ros);
  }
}

pcl::PointCloud<PointType>::Ptr cloud_segmentation::roi_rectangle_filter(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt)
{
  // // Create the filtering object: downsample the dataset using a leaf size
  pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*laserRosCloudMsg, *input_cloud);
  pcl::PointCloud<PointType>::Ptr cloud_roi(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr input_cloud_filter(new pcl::PointCloud<PointType>);

  if (filter_res > 0)
  {
      // Create the filtering object: downsample the dataset using a leaf size
      pcl::VoxelGrid<PointType> vg;
      vg.setInputCloud(input_cloud);
      vg.setLeafSize(filter_res, filter_res, filter_res);
      vg.filter(*input_cloud_filter);

      // Cropping the ROI
      pcl::CropBox<PointType> roi_region(true);
      roi_region.setMin(min_pt);
      roi_region.setMax(max_pt);
      roi_region.setInputCloud(input_cloud_filter);
      roi_region.filter(*cloud_roi);
  }
  else
  {
      // Cropping the ROI
      pcl::CropBox<PointType> roi_region(true);
      roi_region.setMin(min_pt);
      roi_region.setMax(max_pt);
      roi_region.setInputCloud(input_cloud);
      roi_region.filter(*cloud_roi);
  }

  // Removing the car roof region
  std::vector<int> indices;
  pcl::CropBox<PointType> roof(true);

  roof.setMin(Eigen::Vector4f(-1.63, -0.6, -1.86, 1));
  roof.setMax(Eigen::Vector4f(0.97, 0.6, 0.19, 1));

  roof.setInputCloud(cloud_roi);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto& point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);

  return input_cloud_filter;
}

pcl::PointCloud<PointType>::Ptr cloud_segmentation::roi_filter_pcl(const sensor_msgs::PointCloud2::ConstPtr& laserRosCloudMsg, const float filter_res)
{
  float horizonAngle, range;
  size_t  cloudSize;
  int j=0; 
  pcl::PointCloud<PointType>::Ptr roi_cloud_origin(new pcl::PointCloud<PointType>);
  PointType thisPoint;

  // ROS message transform to PCL type
  pcl::PointCloud<PointType>::Ptr origin_cloud_pcl(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*laserRosCloudMsg, *origin_cloud_pcl);

  cloudSize = origin_cloud_pcl->points.size();

  //ROS_INFO("cloudSize=%d",cloudSize);

  // ROI pi/2 ~ -pi/2
  for (size_t i = 0; i < cloudSize; ++i)
  {

    thisPoint.x = origin_cloud_pcl->points[i].x;
    thisPoint.y = origin_cloud_pcl->points[i].y;
    thisPoint.z = origin_cloud_pcl->points[i].z;

    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

    if (range < 1 || range > 3 || (horizonAngle > -180 && horizonAngle < -90))
      continue;

    roi_cloud_origin->push_back(thisPoint);

    j++;
  }
  ROS_INFO("TEST");

  // // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointType> vg;
  pcl::PointCloud<PointType>::Ptr roi_cloud_filter(new pcl::PointCloud<PointType>);
  vg.setInputCloud(roi_cloud_origin);
  vg.setLeafSize(filter_res, filter_res, filter_res);
  vg.filter(*roi_cloud_filter);

  return roi_cloud_filter;
}

void cloud_segmentation::dbscan_kdtree_origin(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud_ptr,
autoware_msgs::CloudClusterArray &in_out_clusters, const float CorePointMinPt, const float ClusterTolerance, const float MinClusterSize, const float MaxClusterSize)
{
  std::cout<<"dbscan_kdtree_origin"<<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  if (in_cloud_ptr->points.size() > 0)
    tree->setInputCloud(in_cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;

  dbscan_kdtree->setCorePointMinPts(CorePointMinPt);
  dbscan_kdtree->setClusterTolerance(ClusterTolerance);
  dbscan_kdtree->setMinClusterSize(MinClusterSize);
  dbscan_kdtree->setMaxClusterSize(MaxClusterSize);
  dbscan_kdtree->setSearchMethod(tree);
  dbscan_kdtree->setInputCloud(in_cloud_ptr);
  dbscan_kdtree->extract(cluster_indices);


  // generate_path(intensity_cloud_ptr, cluster_indices, pubPolyMarkerArray);

  unsigned int k = 0;
  int intensity_mark = 1;
  
  std::vector<ClusterPtr> segment_clusters;
  pcl::PointXYZI cluster_color;

  std::cout<< "=======================================" <<std::endl;
  std::cout<< "Index Count : " << cluster_indices.size() <<std::endl;
  std::cout<< "=======================================" <<std::endl;


  for (auto& getIndices : cluster_indices)
  {
    for (auto& index : getIndices.indices){
      // cluster->points.push_back(cloud->points[index]);
      cluster_color.x=in_cloud_ptr->points[index].x;
      cluster_color.y=in_cloud_ptr->points[index].y;
      cluster_color.z=in_cloud_ptr->points[index].z;
      //cluster_color.intensity=intensity_mark;
      segmentedCloudColor->push_back(cluster_color);
      segmentedCloudColor->points.back().intensity = intensity_mark;
    }

    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, getIndices.indices, _velodyne_header, k, 255,
                      0,
                      0, "", _pose_estimation);
    // cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, 1, 1, 1, "", _pose_estimation);
    segment_clusters.push_back(cluster);
    intensity_mark++;
    k++;
  }
  std::cout<< "=======================================" <<std::endl;
  std::cout<< "Cluster Count : " << segment_clusters.size() <<std::endl;
  std::cout<< "=======================================" <<std::endl;

  for (const auto& cluster_ptr : segment_clusters) {
    // 포인터를 역참조하여 Cluster 객체에 접근합니다.
    Cluster cluster = *cluster_ptr;

    // 포인트 클라우드의 중심점을 계산합니다.
    pcl::PointXYZ center_points;
    center_points=cluster.GetCentroid();

    // 중심점을 출력합니다.
    std::cout << " x: " << center_points.x 
              << " y: " << center_points.y 
              << " z: " << center_points.z << std::endl; 
  }


  for (unsigned int i = 0; i < segment_clusters.size(); i++)
  {
    if (segment_clusters[i]->IsValid())
    {
      autoware_msgs::CloudCluster cloud_cluster;
      segment_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
      in_out_clusters.clusters.push_back(cloud_cluster);
    }
  }
}

void cloud_segmentation::publish_autoware_cloudclusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                          const std::string &in_target_frame, const std_msgs::Header &in_header)
{
  std::cout<<"publish_autoware_cloudclusters"<<std::endl;
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++)
    {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try
      {
        _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(),
                                             *_transform);
        pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                                            cluster_transformed.centroid_point);

        cluster_transformed.dimensions = i->dimensions;
        cluster_transformed.eigen_values = i->eigen_values;
        cluster_transformed.eigen_vectors = i->eigen_vectors;

        cluster_transformed.convex_hull = i->convex_hull;
        cluster_transformed.bounding_box.pose.position = i->bounding_box.pose.position;
        if(_pose_estimation)
        {
          cluster_transformed.bounding_box.pose.orientation = i->bounding_box.pose.orientation;
        }
        else
        {
          cluster_transformed.bounding_box.pose.orientation.w = _initial_quat_w;
        }
        clusters_transformed.clusters.push_back(cluster_transformed);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    //publishAutowareDetectedObjects(clusters_transformed);
  } else
  {
    in_publisher->publish(in_clusters);
    //publishAutowareDetectedObjects(in_clusters);
  }
  in_publisher->publish(in_clusters);
  //publishAutowareDetectedObjects(in_clusters);
}

void cloud_segmentation::publish_ROI_area(const std_msgs::Header& header)
{
  std::cout<<"publish_ROI_area"<<std::endl;
  // Construct Bounding Boxes from the clusters
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  jsk_bboxes.header = header;

  jsk_recognition_msgs::BoundingBox DtectionArea_car;
  DtectionArea_car.pose.position.x=0;
  DtectionArea_car.pose.position.y=0; 
  DtectionArea_car.pose.position.z=0;
  // DtectionArea_car.dimensions.x=ROI_MAX_POINT[1]-ROI_MIN_POINT[1];
  // DtectionArea_car.dimensions.y=ROI_MAX_POINT[2]-ROI_MIN_POINT[2];
  // DtectionArea_car.dimensions.z=4;

  DtectionArea_car.dimensions.x=10;
  DtectionArea_car.dimensions.y=6;
  DtectionArea_car.dimensions.z=4;

  DtectionArea_car.header.frame_id="os_sensor";

  jsk_bboxes.boxes.push_back(DtectionArea_car);

  jsk_recognition_msgs::BoundingBox car_remove;
  car_remove.pose.position.x=0;
  car_remove.pose.position.y=0; 
  car_remove.pose.position.z=0;
  car_remove.dimensions.x=0.97+1.63;
  car_remove.dimensions.y=1.2;
  car_remove.dimensions.z=0.19+1.86;
  car_remove.header.frame_id="os_sensor";
  jsk_bboxes.boxes.push_back(car_remove);
  
  _pub_roi_area.publish(jsk_bboxes);
}

void cloud_segmentation::lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  std::cout<<"lidarPointsCallback"<<std::endl;
  _velodyne_header = lidar_points->header;
  autoware_msgs::CloudClusterArray cloud_clusters;
  cloud_clusters.header = _velodyne_header;

  //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::PointCloud<PointType>::Ptr pcl_roi(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr original_data(new pcl::PointCloud<pcl::PointXYZI>);

  // ROI
  // pcl_roi= roi_filter_pcl(lidar_points, VOXEL_GRID_SIZE);
  // pcl_roi= roi_rectangle_filter(lidar_points, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);

  pcl::fromROSMsg(*lidar_points, *pcl_roi);
  // pcl::fromROSMsg(*lidar_points, *original_data);
  {
    int pointBytes = lidar_points->point_step;
    int offset_x;
    int offset_y;
    int offset_z;
    int offset_int;
    for (int f=0; f<lidar_points->fields.size(); ++f)
    {
      if (lidar_points->fields[f].name == "x")
        offset_x = lidar_points->fields[f].offset;
      if (lidar_points->fields[f].name == "y")
        offset_y = lidar_points->fields[f].offset;
      if (lidar_points->fields[f].name == "z")
        offset_z = lidar_points->fields[f].offset;
      if (lidar_points->fields[f].name == "intensity")
        offset_int = lidar_points->fields[f].offset;
    }

    // populate point cloud object
    for (int p=0; p<lidar_points->width; ++p)
    {
        pcl::PointXYZI newPoint;

        newPoint.x = *(float*)(&(lidar_points->data[0]) + (pointBytes*p) + offset_x);
        newPoint.y = *(float*)(&(lidar_points->data[0]) + (pointBytes*p) + offset_y);
        newPoint.z = *(float*)(&(lidar_points->data[0]) + (pointBytes*p) + offset_z);
        newPoint.intensity = *(unsigned char*)(&(lidar_points->data[0]) + (pointBytes*p) + offset_int);

        original_data->points.push_back(newPoint);
    }
  }

  const auto start_time = std::chrono::steady_clock::now();

  //clustering
  dbscan_kdtree_origin(pcl_roi, original_data, cloud_clusters, CorePointMinPt, ClusterTolerance, MinClusterSize, MaxClusterSize);

  // // Time the whole process
  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;

  publishSegmentedCloudsColor(_velodyne_header);

  resetParameters();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "descan_segment_origin_node");

  cloud_segmentation cloud_segmentation_node;

  ros::spin();

  return 0;
}