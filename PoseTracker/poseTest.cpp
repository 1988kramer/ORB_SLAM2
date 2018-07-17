#include<iostream>
#include<algorithm>
#include<thread>

#include<glog/logging.h>
#

#include "PoseTracker.h"

/*----------------COMMAND LINE FLAGS-------------------*/
DEFINE_string(cam, "", "camera device");
DEFINE_string(settings, "", "path to config file");
DEFINE_string(vocab, "Vocabulary/ORBvoc.txt", "path to vocabulary file");
/*-----------------------------------------------------*/

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = 1;

  PoseTracker tracker(FLAGS_cam, true, FLAGS_settings, FLAGS_vocab, true);
  std::shared_ptr<hal::ImageArray> images;
  Eigen::Matrix4d pose;
  while (tracker.Capture(pose, images))
  {
    VLOG(0) << "got pose: " << pose;
  }
}
