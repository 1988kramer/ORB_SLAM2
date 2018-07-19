#include<iostream>
#include<algorithm>
#include<thread>
#include "SendNode.h"

#include<glog/logging.h>

#include "PoseTracker.h"

/*----------------COMMAND LINE FLAGS-------------------*/
DEFINE_string(cam, "", "camera device");
DEFINE_string(settings, "", "path to config file");
DEFINE_string(vocab, "Vocabulary/ORBvoc.txt", "path to vocabulary file");
/*-----------------------------------------------------*/

void matToEuler(cv::Mat &rot, double &roll, double &pitch, double &yaw);

int main(int argc, char **argv)
{
  char* server_ip = "192.168.50.229";
  unsigned int port = 21234;
  SendNode node(server_ip, port);
  if (!node.ConnectToServer())
    exit(1);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  PoseTracker tracker(FLAGS_cam, true, FLAGS_settings, FLAGS_vocab, true);
  std::shared_ptr<hal::ImageArray> images;
  Eigen::Matrix4d pose;
  dobule roll, pitch, yaw;
  while (tracker.Capture(pose, images))
  {
    matToEuler(pose.rowRange(0,3).colRange(0,3), roll, pitch, yaw);
    node.sendPose(pose.at<double>(1,3),pose.at<double>(2,3),yaw);
  }
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}

void matToEuler(cv::Mat &rot, double &roll, double &pitch, double &yaw)
{
  assert(isRotationMatrix(rot));
     
  float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
  bool singular = sy < 1e-6; // If
 
  float x, y, z;
  if (!singular)
  {
    roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
    pitch = atan2(-R.at<double>(2,0), sy);
    yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }
  else
  {
    roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
    pitch = atan2(-R.at<double>(2,0), sy);
    yaw = 0;
  }
} 
