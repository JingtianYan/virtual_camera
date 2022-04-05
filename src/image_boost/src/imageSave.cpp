#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

const double PI = 3.1415926;

string image_dir;
double disThre = 0.5;
double angThre = 0.5;

struct PosePoint {
     float x, y, z;
     float roll, pitch, yaw;
     double time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PosePoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (double, time, time))

pcl::PointCloud<PosePoint>::Ptr pose(new pcl::PointCloud<PosePoint>());

pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZ>());

PosePoint poseCur, poseRec;

int imageWidth = 640;
int imageHeight = 360;
int imagePixelNum = imageWidth * imageHeight;

bool saveImage = false;
int imageCount = 0;

double rgbImageTime = 0;
double depthImageTime = 0;

cv_bridge::CvImageConstPtr rgbImageCv, depthImageCv;

void rgbImageHandler(const sensor_msgs::Image::ConstPtr& imageIn)
{
  rgbImageTime = imageIn->header.stamp.toSec();
  rgbImageCv = cv_bridge::toCvShare(imageIn, "bgr8");
}

void depthImageHandler(const sensor_msgs::Image::ConstPtr& imageIn)
{
  depthImageTime = imageIn->header.stamp.toSec();
  depthImageCv = cv_bridge::toCvShare(imageIn, "32FC1");
}

/*void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
{
  depthCloud->clear();
  pcl::fromROSMsg(*cloudIn, *depthCloud);
  int depthCloudSize = depthCloud->points.size();
  
  double minFx = 0, maxFx = 0, minFy = 0, maxFy = 0;
  for (int i = 0; i < depthCloudSize; i++) {
    double fx = double(depthCloud->points[i].x) / double(depthCloud->points[i].z) / double(imageWidth / 2 - 0.5);
    double fy = double(depthCloud->points[i].y) / double(depthCloud->points[i].z) / double(imageHeight / 2 - 0.5);
    
    if (fx < minFx) minFx = fx;
    if (fx > maxFx) maxFx = fx;
    if (fy < minFy) minFy = fy;
    if (fy > maxFy) maxFy = fy;
  }
  
  printf ("minFx: %f, maxFx: %f, minFy: %f, maxFy: %f\n", 1.0 / minFx, 1.0 / maxFx, 1.0 / minFy, 1.0 / maxFy);
}*/

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  double odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  poseCur.x = odomIn->pose.pose.position.x;
  poseCur.y = odomIn->pose.pose.position.y;
  poseCur.z = odomIn->pose.pose.position.z;
  poseCur.roll = roll;
  poseCur.pitch = pitch;
  poseCur.yaw = yaw;
  poseCur.time = odomTime;

  float disX = poseCur.x - poseRec.x;
  float disY = poseCur.y - poseRec.y;
  float disZ = poseCur.z - poseRec.z;
  float dis = sqrt(disX * disX + disY * disY + disZ * disZ);

  float disRoll = poseCur.roll - poseRec.roll;
  float disPitch = poseCur.pitch - poseRec.pitch;
  float disYaw = poseCur.yaw - poseRec.yaw;
  if (disYaw < -PI) disYaw += 2 * PI;
  else if (disYaw > PI) disYaw -= 2 * PI;
  float disAng = sqrt(disRoll * disRoll + disPitch * disPitch + disYaw * disYaw);

  if (dis > disThre || disAng > angThre) {
    poseRec = poseCur;
    saveImage = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imageSave");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("image_dir", image_dir);
  nhPrivate.getParam("disThre", disThre);
  nhPrivate.getParam("angThre", angThre);

  ros::Subscriber rgbImageSub = nh.subscribe<sensor_msgs::Image> ("/rgbd_camera/color/image", 1, rgbImageHandler);

  ros::Subscriber depthImageSub = nh.subscribe<sensor_msgs::Image> ("/rgbd_camera/depth/image", 1, depthImageHandler);

  //ros::Subscriber depthCloudSub = nh.subscribe<sensor_msgs::PointCloud2> ("/rgbd_camera/depth/points", 5, depthCloudHandler);

  ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry> ("/state_estimation_at_image", 5, odomHandler);

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (saveImage && fabs(rgbImageTime - depthImageTime) < 0.005 && rgbImageTime > poseRec.time && rgbImageTime > 0) {
      saveImage = false;

      imwrite(image_dir + "/rgb_" + to_string(imageCount) + ".png", rgbImageCv->image);
      
      string fileName = image_dir + "/depth_" + to_string(imageCount) + ".bin";
      FILE* fileVar = fopen(fileName.c_str(), "w");
      fwrite(depthImageCv->image.data, 4, imagePixelNum, fileVar);
      fclose(fileVar);

      printf ("Saved image: %d\n", imageCount);
      imageCount++;

      pose->push_back(poseRec);
    }
    
    status = ros::ok();
    rate.sleep();
  }

  string fileName = image_dir + "/pose.ply";
  FILE* fileVar = fopen(fileName.c_str(), "w");
  int poseSize = pose->points.size();

  fprintf(fileVar, "ply\n");
  fprintf(fileVar, "format ascii 1.0\n");
  fprintf(fileVar, "element vertex %d\n", poseSize);
  fprintf(fileVar, "property float x\n");
  fprintf(fileVar, "property float y\n");
  fprintf(fileVar, "property float z\n");
  fprintf(fileVar, "property float roll\n");
  fprintf(fileVar, "property float pitch\n");
  fprintf(fileVar, "property float yaw\n");
  fprintf(fileVar, "property double time\n");
  fprintf(fileVar, "end_header\n");

  for (int i = 0; i < poseSize; i++) {
      fprintf(fileVar, "%f %f %f %f %f %f %f\n", pose->points[i].x, pose->points[i].y, pose->points[i].z, 
              pose->points[i].roll, pose->points[i].pitch, pose->points[i].yaw, pose->points[i].time);
  }

  fclose(fileVar);

  return 0;
}
