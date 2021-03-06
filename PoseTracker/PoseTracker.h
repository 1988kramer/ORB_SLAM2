// Uses ORBSLAM2 to track the pose of a monocular or stereo camera
// relative to the camera's starting pose.
// Allows user to query the camera's pose at any time.

#pragma once
#ifndef POSETRACKER_H
#define POSE_TRACKER_H

#include<algorithm>
#include<fstream>
#include<string.h>

#include<HAL/Camera/CameraDevice.h>
#include<HAL/Messages/Matrix.h>
#include<glog/logging.h>

#include<System.h>

class PoseTracker
{
public:
  PoseTracker(string cam_uri, bool monocular, string config, string vocab, bool use_viewer = false);
  ~PoseTracker();

  bool Capture(Eigen::Matrix4d &pose,
               std::shared_ptr<hal::ImageArray> images); // adds new images to the SLAM system and updates the pose estimate
                         // should be run in a separate thread from other functions
private:
  bool LoadCameras(); // loads camera (or cameras) from the uri's provided in the constructor
  bool LoadRectParams();

  hal::Camera cam_;
  Eigen::Matrix4d pose_;
  string cam_uri_;
  bool monocular_;
  string vocabFile_;
  ORB_SLAM2::System *SLAMSystem_;
  cv::Mat M1l_, M2l_, M1r_, M2r_; // stereo rectification matrices
};

#endif
