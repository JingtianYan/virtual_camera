#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

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
double imageLoadDis = 10.0;
double maxDepth = 29.5;
double maxDepthMatchDiff = 0.2;
double maxDepthPixelDiff = 0.2;
double minAngleMatchDiff = 1.0;
double angleToDisGain = 10.0;
double fx = 295.608024;
double fy = 295.608024;
double cx = 319.5;
double cy = 179.5;

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

PosePoint poseCur, poseLast, poseLLast, poseMatched;

int imageWidth = 640;
int imageHeight = 360;
int imagePixelNum = imageWidth * imageHeight;
int imageSurroundNum = 100;

bool newRgbImage = false;
bool newDepthImage = false;
double rgbImageTime = 0;
double depthImageTime = 0;

cv_bridge::CvImageConstPtr rgbImageCv, depthImageCv;

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readPoses()
{
  string fileName = image_dir + "/pose.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  PosePoint point;
  int val1, val2, val3, val4, val5, val6, val7;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%f", &point.roll);
    val5 = fscanf(filePtr, "%f", &point.pitch);
    val6 = fscanf(filePtr, "%f", &point.yaw);
    val7 = fscanf(filePtr, "%lf", &point.time);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    pose->push_back(point);
  }

  fclose(filePtr);
}

void rgbImageHandler(const sensor_msgs::Image::ConstPtr& imageIn)
{
  rgbImageTime = imageIn->header.stamp.toSec();
  rgbImageCv = cv_bridge::toCvShare(imageIn, "bgr8");
  newRgbImage = true;
}

void depthImageHandler(const sensor_msgs::Image::ConstPtr& imageIn)
{
  depthImageTime = imageIn->header.stamp.toSec();
  depthImageCv = cv_bridge::toCvShare(imageIn, "32FC1");
  newDepthImage = true;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  double odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  poseLLast = poseLast;
  poseLast = poseCur;

  poseCur.x = odomIn->pose.pose.position.x;
  poseCur.y = odomIn->pose.pose.position.y;
  poseCur.z = odomIn->pose.pose.position.z;
  poseCur.roll = roll;
  poseCur.pitch = pitch;
  poseCur.yaw = yaw;
  poseCur.time = odomTime;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imageBoost");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("image_dir", image_dir);
  nhPrivate.getParam("imageLoadDis", imageLoadDis);
  nhPrivate.getParam("maxDepth", maxDepth);
  nhPrivate.getParam("maxDepthMatchDiff", maxDepthMatchDiff);
  nhPrivate.getParam("maxDepthPixelDiff", maxDepthPixelDiff);
  nhPrivate.getParam("minAngleMatchDiff", minAngleMatchDiff);
  nhPrivate.getParam("angleToDisGain", angleToDisGain);
  nhPrivate.getParam("fx", fx);
  nhPrivate.getParam("fy", fy);
  nhPrivate.getParam("cx", cx);
  nhPrivate.getParam("cy", cy);

  ros::Subscriber rgbImageSub = nh.subscribe<sensor_msgs::Image> ("/rgbd_camera/color/image", 1, rgbImageHandler);

  ros::Subscriber depthImageSub = nh.subscribe<sensor_msgs::Image> ("/rgbd_camera/depth/image", 1, depthImageHandler);

  ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry> ("/state_estimation_at_image", 5, odomHandler);

  readPoses();
  int poseSize = pose->points.size();

  int* poseStatus = new int[poseSize];
  for (int i = 0; i < poseSize; i++) {
    poseStatus[i] = 0;
  }

  Mat* rgbImageSurround = new Mat[imageSurroundNum];
  Mat* depthImageSurround = new Mat[imageSurroundNum];

  int* imageSurroundInd = new int[imageSurroundNum];
  float* sinRollSurround = new float[imageSurroundNum];
  float* cosRollSurround = new float[imageSurroundNum];
  float* sinPitchSurround = new float[imageSurroundNum];
  float* cosPitchSurround = new float[imageSurroundNum];
  float* sinYawSurround = new float[imageSurroundNum];
  float* cosYawSurround = new float[imageSurroundNum];
  
  for (int i = 0; i < imageSurroundNum; i++) {
    imageSurroundInd[i] = -1;
    sinRollSurround[i] = 0;
    cosRollSurround[i] = 0;
    sinPitchSurround[i] = 0;
    cosPitchSurround[i] = 0;
    sinYawSurround[i] = 0;
    cosYawSurround[i] = 0;
  }

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newRgbImage && newDepthImage && fabs(rgbImageTime - depthImageTime) < 0.005 && rgbImageTime > poseLLast.time && poseLLast.time > 0) {
      newRgbImage = false;
      newDepthImage = false;

      if (rgbImageTime > poseCur.time) poseMatched = poseCur;
      else if (rgbImageTime > poseLast.time) poseMatched = poseLast; 
      else poseMatched = poseLLast;

      vector<pair<float, int>> imageSortedInd;
      for (int i = 0; i < imageSurroundNum; i++) {
        if (imageSurroundInd[i] >= 0) {
          float disX = poseMatched.x - pose->points[imageSurroundInd[i]].x;
          float disY = poseMatched.y - pose->points[imageSurroundInd[i]].y;
          float disZ = poseMatched.z - pose->points[imageSurroundInd[i]].z;
          float dis = sqrt(disX * disX + disY * disY + disZ * disZ);

          float disRoll = poseMatched.roll - pose->points[imageSurroundInd[i]].roll;
          float disPitch = poseMatched.pitch - pose->points[imageSurroundInd[i]].pitch;
          float disYaw = poseMatched.yaw - pose->points[imageSurroundInd[i]].yaw;
          if (disYaw < -PI) disYaw += 2 * PI;
          else if (disYaw > PI) disYaw -= 2 * PI;
          float disAng = sqrt(disRoll * disRoll + disPitch * disPitch + disYaw * disYaw);

          if (disAng < minAngleMatchDiff) {
            float score = dis + angleToDisGain * disAng;
            imageSortedInd.push_back(make_pair(score, i));
          }
        }
      }

      sort(imageSortedInd.begin(), imageSortedInd.end());
      int imageSortedNum = imageSortedInd.size();

      printf ("Sorted image buffer: %d\n", imageSortedNum);

      float sinRollCur = sin(poseMatched.roll);
      float cosRollCur = cos(poseMatched.roll);
      float sinPitchCur = sin(poseMatched.pitch);
      float cosPitchCur = cos(poseMatched.pitch);
      float sinYawCur = sin(poseMatched.yaw);
      float cosYawCur = cos(poseMatched.yaw);

      Mat rgbImage = Mat(imageHeight, imageWidth, CV_8UC3, Scalar(0, 0, 0));
      for (int i = 0; i < imagePixelNum; i++) {
        int u = i % imageWidth;
        int v = int(i / imageWidth);

        float fwd = depthImageCv->image.at<float>(v, u);
        float left = (cx - u) / fx * fwd;
        float up = (cy - v) / fy * fwd;

        if (fwd > maxDepth) continue;

        float pointX1 = fwd;
        float pointY1 = left * cosRollCur - up * sinRollCur;
        float pointZ1 = left * sinRollCur + up * cosRollCur;

        float pointX2 = pointX1 * cosPitchCur + pointZ1 * sinPitchCur;
        float pointY2 = pointY1;
        float pointZ2 = -pointX1 * sinPitchCur + pointZ1 * cosPitchCur;

        float pointX3 = pointX2 * cosYawCur - pointY2 * sinYawCur + poseMatched.x;
        float pointY3 = pointX2 * sinYawCur + pointY2 * cosYawCur + poseMatched.y;
        float pointZ3 = pointZ2 + poseMatched.z;

        for (int imageRefInd = 0; imageRefInd < imageSortedNum; imageRefInd++) {
          int surroundID = imageSortedInd[imageRefInd].second;
          int poseID = imageSurroundInd[surroundID];

          float sinRollRef = sinRollSurround[surroundID];
          float cosRollRef = cosRollSurround[surroundID];
          float sinPitchRef = sinPitchSurround[surroundID];
          float cosPitchRef = cosPitchSurround[surroundID];
          float sinYawRef = sinYawSurround[surroundID];
          float cosYawRef = cosYawSurround[surroundID];

          float pointX4 = (pointX3 - pose->points[poseID].x) * cosYawRef + (pointY3 - pose->points[poseID].y) * sinYawRef;
          float pointY4 = -(pointX3 - pose->points[poseID].x) * sinYawRef + (pointY3 - pose->points[poseID].y) * cosYawRef;
          float pointZ4 = pointZ3 - pose->points[poseID].z;

          float pointX5 = pointX4 * cosPitchRef - pointZ4 * sinPitchRef;
          float pointY5 = pointY4;
          float pointZ5 = pointX4 * sinPitchRef + pointZ4 * cosPitchRef;

          float fwd2 = pointX5;
          float left2 = pointY5 * cosRollRef + pointZ5 * sinRollRef;
          float up2 = -pointY5 * sinRollRef + pointZ5 * cosRollRef;

          int u2 = cx - fx * left2 / fwd2 + 0.5;
          int v2 = cy - fy * up2 / fwd2 + 0.5;

          bool depthMatched = false;
          if (u2 >= 0 && u2 < imageWidth && v2 >= 0 && v2 < imageHeight && fwd2 > 0) {
            float fwdRef2 = depthImageSurround[surroundID].at<float>(v2, u2);
            if (fabs(fwd2 - fwdRef2) < maxDepthMatchDiff && fwdRef2 < maxDepth) {
              depthMatched = true; 
            } else {
              if (u2 >= 1 && u2 < imageWidth - 1 && v2 >= 1 && v2 < imageHeight - 1 && fwd2 > 0) {
                for (int du = -1; du <= 1; du++) {
                  for (int dv = -1; dv <= 1; dv++) {
                    float fwdRef2 = depthImageSurround[surroundID].at<float>(v2 + dv, u2 + du);
                    if (fabs(fwd2 - fwdRef2) < maxDepthMatchDiff && fwdRef2 < maxDepth) {
                      depthMatched = true; 
                      break;
                    }
                  }
                  if (depthMatched) break;
                }

                if (depthMatched) {
                  float fwdDiff1 = fabs(2 * depthImageSurround[surroundID].at<float>(v2, u2) 
                                 - depthImageSurround[surroundID].at<float>(v2 - 1, u2) - depthImageSurround[surroundID].at<float>(v2 + 1, u2));
                  float fwdDiff2 = fabs(2 * depthImageSurround[surroundID].at<float>(v2, u2)
                                 - depthImageSurround[surroundID].at<float>(v2, u2 - 1) - depthImageSurround[surroundID].at<float>(v2, u2 + 1));
                  float fwdDiff3 = fabs(2 * depthImageSurround[surroundID].at<float>(v2, u2)
                                 - depthImageSurround[surroundID].at<float>(v2 - 1, u2 - 1) - depthImageSurround[surroundID].at<float>(v2 + 1, u2 + 1));
                  float fwdDiff4 = fabs(2 * depthImageSurround[surroundID].at<float>(v2, u2)
                                 - depthImageSurround[surroundID].at<float>(v2 - 1, u2 + 1) - depthImageSurround[surroundID].at<float>(v2 + 1, u2 - 1));
                  if (fwdDiff1 > maxDepthPixelDiff || fwdDiff2 > maxDepthPixelDiff || fwdDiff3 > maxDepthPixelDiff || fwdDiff4 > maxDepthPixelDiff) {
                    depthMatched = false;
                  }
                }
              }
            }

            if (depthMatched) {
              int j = imageWidth * v2 + u2;
              rgbImage.data[3 * i] = rgbImageSurround[surroundID].data[3 * j];
              rgbImage.data[3 * i + 1] = rgbImageSurround[surroundID].data[3 * j + 1];
              rgbImage.data[3 * i + 2] = rgbImageSurround[surroundID].data[3 * j + 2];
            }
          }
          if (depthMatched) break;
        }
      }

      for (int i = 0; i < imageSurroundNum; i++) {
        if (imageSurroundInd[i] >= 0) {
          float disX = poseMatched.x - pose->points[imageSurroundInd[i]].x;
          float disY = poseMatched.y - pose->points[imageSurroundInd[i]].y;
          float disZ = poseMatched.z - pose->points[imageSurroundInd[i]].z;
          float dis = sqrt(disX * disX + disY * disY + disZ * disZ);

          if (dis > imageLoadDis) {
            rgbImageSurround[i].release();
            depthImageSurround[i].release();
            
            poseStatus[imageSurroundInd[i]] = 0;
            imageSurroundInd[i] = -1;
          }
        }
      }

      for (int i = 0; i < poseSize; i++) {
        float disX = poseMatched.x - pose->points[i].x;
        float disY = poseMatched.y - pose->points[i].y;
        float disZ = poseMatched.z - pose->points[i].z;
        float dis = sqrt(disX * disX + disY * disY + disZ * disZ);
        
        if (dis <= imageLoadDis && poseStatus[i] == 0) {
          for (int j = 0; j < imageSurroundNum; j++) {
            if (imageSurroundInd[j] == -1) {
              rgbImageSurround[j] = imread(image_dir + "/rgb_" + to_string(i) + ".png");
              depthImageSurround[j] = Mat(imageHeight, imageWidth, CV_32FC1, Scalar(0, 0, 0));

              string fileName = image_dir + "/depth_" + to_string(i) + ".bin";
              FILE* fileVar = fopen(fileName.c_str(), "r");
              int num = fread(depthImageSurround[j].data, 4, imagePixelNum, fileVar);
              fclose(fileVar);
    
              if (num != imagePixelNum) printf("Image size error, read %d/%d numbers.\n", num, imagePixelNum);

              sinRollSurround[j] = sin(pose->points[i].roll);
              cosRollSurround[j] = cos(pose->points[i].roll);
              sinPitchSurround[j] = sin(pose->points[i].pitch);
              cosPitchSurround[j] = cos(pose->points[i].pitch);
              sinYawSurround[j] = sin(pose->points[i].yaw);
              cosYawSurround[j] = cos(pose->points[i].yaw);

              poseStatus[i] = 1;
              imageSurroundInd[j] = i;
              
              break;
            }
          }
        }
      }

      imshow("Image", rgbImage);
      waitKey(10);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
