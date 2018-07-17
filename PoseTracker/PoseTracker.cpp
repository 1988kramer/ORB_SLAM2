#include "PoseTracker.h"


PoseTracker::PoseTracker(string cam_uri, bool monocular, string config, string vocab, bool use_viewer)
{
  monocular_ = monocular;
  cam_uri_ = cam_uri;
  VLOG(3) << "loading camera(s)";
  LoadCameras();
  VLOG(3) << "initializing SLAM system";
  if (monocular_)
    SLAMSystem_ = new ORB_SLAM2::System(vocab, config, ORB_SLAM2::System::MONOCULAR, use_viewer);
  else
    SLAMSystem_ = new ORB_SLAM2::System(vocab, config, ORB_SLAM2::System::STEREO, use_viewer);
  VLOG(3) << "SLAM system initialized";
}

PoseTracker::~PoseTracker()
{
  SLAMSystem_->Shutdown();
  delete SLAMSystem_;
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
    cv::Mat im0, im1;
    im0 = images->at(0)->Mat().clone(); // get images from HAL array
    im1 = images->at(1)->Mat().clone();
    double timestamp = images->Ref().device_time(); // get image timestamp in ns
    if (im0.empty() || im1.empty()) // check if either image is empty
    {
      VLOG(0) << "captured stereo image with timestamp: " << timestamp << " is empty";
      return false;
    }
    timestamp /= 1e9; // convert timestamp from ns to seconds
    VLOG(3) << "adding stereo image to system with timestamp: " << timestamp;
    mat_pose = SLAMSystem_->TrackStereo(im0, im1, timestamp);
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
