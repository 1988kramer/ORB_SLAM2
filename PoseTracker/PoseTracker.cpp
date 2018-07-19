#include "PoseTracker.h"


PoseTracker::PoseTracker(string cam_uri, bool monocular, string config, string vocab, bool use_viewer)
{
  monocular_ = monocular;
  cam_uri_ = cam_uri;
  VLOG(3) << "loading camera(s)";
  LoadCameras();
  VLOG(3) << "initializing SLAM system";
  if (monocular_)
  {
    SLAMSystem_ = new ORB_SLAM2::System(vocab, config, ORB_SLAM2::System::MONOCULAR, use_viewer);
  }
  else
  {
    SLAMSystem_ = new ORB_SLAM2::System(vocab, config, ORB_SLAM2::System::STEREO, use_viewer);
    LoadRectParams(config);
  }
  VLOG(3) << "SLAM system initialized";
}

PoseTracker::~PoseTracker()
{
  SLAMSystem_->Shutdown();
  delete SLAMSystem_;
}

bool PoseTracker::LoadRectParams(string config)
{
  cv::FileStorage fs_settings(config, cv::FileStorage::READ);
  if (!fs_settings.isOpened())
  {
    VLOG(0) << "could not open config file";
    return false;
  }

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fs_settings["LEFT.K"] >> K_l;
  fs_settings["RIGHT.K"] >> K_r;
  fs_settings["LEFT.P"] >> P_l;
  fs_settings["RIGHT.P"] >> P_r;
  fs_settings["LEFT.R"] >> R_l;
  fs_settings["RIGHT.R"] >> R_r;
  fs_settings["LEFT.D"] >> D_l;
  fs_settings["RIGHT.D"] >> D_r;

  int rows_l = fs_settings["LEFT.height"];
  int cols_l = fs_settings["LEFT.width"];
  int rows_r = fs_settings["RIGHT.height"];
  int cols_r = fs_settings["RIGHT.width"];

  if (K_l.empty() || K_r.empty() || P_l.empty() ||
      P_r.empty() || R_l.empty() || R_r.empty() ||
      D_l.empty() || D_r.empty() || rows_l == 0 ||
      cols_l == 0 || rows_r == 0 || cols_r == 0)
  {
    VLOG(0) << "stereo rectification parameters are missing";
    return false;
  }
  cv::initUndistortRectifyMap(K_l, D_l, R_l,
                              P_l.rowRange(0,3).colRange(0,3),
                              cv::Size(cols_l, rows_l),
                              CV_32F, M1l_, M2l_);
  cv::initUndistortRectifyMap(K_r, D_r, R_r,
                              P_r.rowRange(0,3).colRange(0,3),
                              cv::Size(cols_r, rows_r),
                              CV_32F, M1r_, M2r_);
  return false;
}


bool PoseTracker::Capture(Eigen::Matrix4d &pose,
                          std::shared_ptr<hal::ImageArray> images)
{

  images = hal::ImageArray::Create();
  cv::Mat mat_pose;

  if (!cam_.Capture(*images)) // stop if capturing the next image fails
  {
    VLOG(0) << "failed to capture image";
    return false;
  }
  if (monocular_)
  {
    cv::Mat im;
    im = images->at(0)->Mat().clone(); // get image fram HAL array
    double timestamp = images->Ref().device_time(); // get image timestamp in ns
    if (im.empty()) // check if the obtained image is empty
    {
      VLOG(0) << "captured mono image with timestamp: " << timestamp << " is empty";
      return false;
    }
    timestamp /= 1e9; // convert timestamp from ns to seconds
    VLOG(3) << "adding mono image to system with timestamp: " << timestamp;
    mat_pose = SLAMSystem_->TrackMonocular(im,timestamp);
  }
  else
  {
    // need to add stereo rectification to this process
    cv::Mat im0, im1, im0_rect, im1_rect;
    im0 = images->at(0)->Mat().clone(); // get images from HAL array
    im1 = images->at(1)->Mat().clone();
    VLOG(3) << "rectifying images";
    cv::remap(im0, im0_rect, M1l_, M2l_, cv::INTER_LINEAR);
    cv::remap(im1, im1_rect, M1r_, M2r_, cv::INTER_LINEAR);
    double timestamp = images->Ref().device_time(); // get image timestamp in ns
    if (im0.empty() || im1.empty()) // check if either image is empty
    {
      VLOG(0) << "captured stereo image with timestamp: " << timestamp << " is empty";
      return false;
    }
    timestamp /= 1e9; // convert timestamp from ns to seconds
    VLOG(3) << "adding stereo image to system with timestamp: " << timestamp;
    mat_pose = SLAMSystem_->TrackStereo(im0_rect, im1_rect, timestamp);
  }
  VLOG(2) << "pose mat rows: " << mat_pose.rows;
  VLOG(2) << "pose mat cols: " << mat_pose.cols;
  // if tracking was successful
  // pull values from cv matrix to eigen matrix
  if (!mat_pose.empty())
  {
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
        pose_(i,j) = mat_pose.at<double>(i,j);
    }
  }
  pose = pose_;
  return true;
}

// load cameras from hal URIs
bool PoseTracker::LoadCameras()
{
  // load camera (or cameras)
  try
  {
    cam_ = hal::Camera(hal::Uri(cam_uri_));
  }
  catch (hal::DeviceException &e)
  {
    LOG(ERROR) << "Error loading camera device: " << e.what();
    return false;
  }
  return true;
}
