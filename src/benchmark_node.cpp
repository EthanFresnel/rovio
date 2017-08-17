#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#define foreach BOOST_FOREACH

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

void tracePose(std::ofstream& trace_est_pose,
               const Eigen::Vector3d& t,
               const Eigen::Vector4d& q,
               const int64_t id)
{
  trace_est_pose.precision(8);
  trace_est_pose << id << " ";
  trace_est_pose << t(0) << " " << t(1) << " " << t(2) << " "
                 << q(1) << " " << q(2) << " " << q(3) << " "
                 << -q(0) << " " << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rovio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string rootdir = ros::package::getPath("rovio"); // Leaks memory
  std::string filter_config = rootdir + "/cfg/rovio.info";

  nh_private.param("filter_config", filter_config, filter_config);

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (nh_private.getParam("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  mpFilter->refreshProperties();

  // Node
  rovio::RovioNode<mtFilter> rovioNode(nh, nh_private, mpFilter);
  rovioNode.makeTest();
  double resetTrigger = 0.0;

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);
  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  std::string trace_dir = "/tmp";
  nh_private.param("trace_dir", trace_dir, trace_dir);
  std::string filename_out = trace_dir + "/traj_estimate.txt";
  std::ofstream trace_est_pose;
  trace_est_pose.open(filename_out.c_str());
  if(trace_est_pose.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");
  std::cout << "Writing trace of estimated pose to: " << filename_out << std::endl;

  // Copy info
  std::string info_filename_out = trace_dir + "/filter_config.info";
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();

  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  nh_private.param("imu_topic_name", imu_topic_name, imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  nh_private.param("cam0_topic_name", cam0_topic_name, cam0_topic_name);

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));

  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  int64_t img_id = -1;
  bool new_pose = false;
  for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++){
    if(it->getTopic() == imu_topic_name){
      sensor_msgs::Imu::ConstPtr imuMsg = it->instantiate<sensor_msgs::Imu>();
      if (imuMsg != NULL) rovioNode.imuCallback(imuMsg);
    }
    if(it->getTopic() == cam0_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      ++img_id;
      new_pose = true;
      if (imgMsg != NULL) rovioNode.imgCallback0(imgMsg);
    }
    ros::spinOnce();

    if(rovioNode.gotFirstMessages_){
      static double lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      if(rovioNode.mpFilter_->safe_.t_ > lastSafeTime){
        if(new_pose)
        {
          tracePose(trace_est_pose, rovioNode.imuOutput_.WrWB(), rovioNode.imuOutput_.qBW().vector(), img_id);
          new_pose = false;
        }

        lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovioNode.requestReset();
        rovioNode.mpFilter_->init_.state_.WrWM() = rovioNode.mpFilter_->safe_.state_.WrWM();
        rovioNode.mpFilter_->init_.state_.qWM() = rovioNode.mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagIn.close();
  trace_est_pose.close();


  return 0;
}
