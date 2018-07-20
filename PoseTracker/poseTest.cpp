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
    Eigen::Matrix4d world_to_car = pose.inverse();
    matToEuler(world_to_car.block<0,0>(3,3), roll, pitch, yaw);
    node.sendPose(world_to_car(2,3), -1.0 * world_to_car(0,3),-1.0*yaw);
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

void matToEuler(Eigen::Matrix3d &rot, double &roll, double &pitch, double &yaw)
{
  assert(isRotationMatrix(rot));
     
  float sy = sqrt(rot(0,0) * rot(0,0) +  rot(1,0) * rot(1,0) );
 
  bool singular = sy < 1e-6; // If
 
  if (!singular)
  {
    pitch = atan2(rot(2,1) , rot(2,2));
    yaw = atan2(-rot(2,0), sy);
    roll = atan2(rot(1,0), rot(0,0));
  }
  else
  {
    pitch = atan2(-rot(1,2), rot(1,1));
    yaw = atan2(-rot(2,0), sy);
    roll = 0;
  }
} 
