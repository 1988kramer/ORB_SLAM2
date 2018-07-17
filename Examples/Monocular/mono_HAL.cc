/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>

#include<HAL/Camera/CameraDevice.h>
#include<HAL/Messages/Matrix.h>
#include<glog/logging.h>

#include<System.h>

using namespace std;

hal::Camera camera_device;

/*-----------------COMMAND LINE FLAGS--------------------*/
DEFINE_string(cam, "", "camera device");
DEFINE_string(settings, "", "path to config file");
DEFINE_string(vocab, "Vocabulary/ORBvoc.txt", "path to vocabulary file");
/*-------------------------------------------------------*/

void LoadCamera();

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  LoadCamera();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(FLAGS_vocab,FLAGS_settings,ORB_SLAM2::System::MONOCULAR,true);

  LOG(INFO) << "Start processing sequence ...";

  // Main loop
  vector<float> vTimesTrack;
  vector<double> vTimestamps;
  int ni = 0;
  cv::Mat im;
  std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
  bool capture_success = true;

  while(true)
  {
    VLOG(3) << "capturing image";
    capture_success = camera_device.Capture(*images);
    if (!capture_success)
      break;
    VLOG(3) << "image captured";
    im = images->at(0)->Mat().clone();
    double tframe = images->Ref().device_time();
    tframe /= 1e9;
    VLOG(3) << "adding timestamp to timestamp log";
    vTimestamps.push_back(tframe);
    if(im.empty())
      LOG(FATAL) << "failed to load image with timestamp: " << tframe;
    VLOG(3) << "got image with timestamp: " << tframe;

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the image to the SLAM system
    VLOG(3) << "adding image to SLAM system";
    SLAM.TrackMonocular(im,tframe);
    VLOG(3) << "image added to SLAM system";
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack.push_back(ttrack);

    // Wait to load the next frame
    double T = vTimestamps[ni+1]-tframe;
    /*
        if(ttrack<T)
  {
    double sleep_time = (T-ttrack)*1e6;
    VLOG(3) << "sleeping for " << sleep_time;
            usleep(sleep_time);
  }
  */
    ni++;
  }

  // Stop all threads
  SLAM.Shutdown();

  int nImages = vTimesTrack.size();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for(int ni=0; ni<nImages; ni++)
  {
    totaltime+=vTimesTrack[ni];
  }
  std::cout << "-------" << std::endl << std::endl;
  std::cout << "median tracking time: " << vTimesTrack[nImages/2] << std::endl;
  std::cout << "mean tracking time: " << totaltime/nImages << std::endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void LoadCamera()
{
  try
  {
    camera_device = hal::Camera(hal::Uri(FLAGS_cam));
  }
  catch (hal::DeviceException &e)
  {
    LOG(ERROR) << "Error loading camera device: " << e.what();
  }
}
