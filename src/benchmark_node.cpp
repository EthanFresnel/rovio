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

#include <unordered_map>
#include <utility>
#include <vikit/timer.h>

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
typedef rovio::RovioNode<mtFilter> rovioNode;

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

void traceTiming(std::ofstream& trace, const double timestamp, const double t_frame)
{
  trace.precision(16);
  trace << timestamp << ", " << t_frame << std::endl;
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
  rovioNode rovio_node(nh, nh_private, mpFilter);
  rovio_node.makeTest();
  double resetTrigger = 0.0;

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);
  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  std::string trace_dir = "/tmp";
  nh_private.param("trace_dir", trace_dir, trace_dir);
  std::string traj_out = trace_dir + "/traj_estimate.txt";
  std::ofstream trace_est_pose;
  trace_est_pose.open(traj_out.c_str());
  if(trace_est_pose.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");
  std::string timing_out = trace_dir + "/trace.csv";
  std::ofstream trace_timing;
  trace_timing.open(timing_out.c_str());
  if(trace_timing.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");
  trace_timing << "timestamp" << "," << "tot_time" << std::endl;
  std::cout << "Writing trace of estimated pose to: " << traj_out << " and timing to: " << timing_out << std::endl;

  // Copy info
  std::string info_filename_out = trace_dir + "/filter_config.info";
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();

  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  nh_private.param("imu_topic_name", imu_topic_name, imu_topic_name);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic_name,1);
  std::string cam0_topic_name = "/cam0/image_raw";
  nh_private.param("cam0_topic_name", cam0_topic_name, cam0_topic_name);
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(cam0_topic_name,1);

  int dataset_last_frame = 10000;
  nh_private.param("dataset_last_frame", dataset_last_frame, dataset_last_frame);

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));

  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  int64_t img_id = -1;
  bool new_pose = false;

  std::unordered_map<double, std::pair<int64_t, vk::Timer>> results;

  ros::Time this_msg_time, last_msg_time;
  bool finished = false;
  for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++){
    ros::spinOnce();
    if(it->getTopic() == imu_topic_name){
      sensor_msgs::Imu::ConstPtr imuMsg = it->instantiate<sensor_msgs::Imu>();
      this_msg_time = imuMsg->header.stamp;
      if (imuMsg != NULL)
        imu_pub.publish(imuMsg);
    }
    if(it->getTopic() == cam0_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      this_msg_time = imgMsg->header.stamp;
      ++img_id;
      new_pose = true;
      if (imgMsg != NULL)
      {
        results.emplace(std::make_pair(imgMsg->header.stamp.toSec(), std::make_pair(img_id, vk::Timer())));
        img_pub.publish(imgMsg);
      }
      if(img_id > dataset_last_frame)
        finished = true;
    }
    if(last_msg_time.toSec() > 0)
      ros::Duration(this_msg_time-last_msg_time).sleep();
    if(!this_msg_time.is_zero())
      last_msg_time = this_msg_time;

    if(rovio_node.gotFirstMessages_){
      static double lastSafeTime = rovio_node.mpFilter_->safe_.t_;
      if(rovio_node.mpFilter_->safe_.t_ > lastSafeTime){
        lastSafeTime = rovio_node.mpFilter_->safe_.t_;

        if(new_pose)
        {
          auto result = results.find(lastSafeTime);
          if(result != results.end())
          {
            tracePose(trace_est_pose, rovio_node.imuOutput_.WrWB(), rovio_node.imuOutput_.qBW().vector(), result->second.first);
            new_pose = false;
            traceTiming(trace_timing, lastSafeTime, result->second.second.stop());
            results.erase(lastSafeTime);
          }
          else
          {
            ROS_WARN_STREAM("Could not find result from time " << lastSafeTime << " in results.");
          }
        }
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovio_node.requestReset();
        rovio_node.mpFilter_->init_.state_.WrWM() = rovio_node.mpFilter_->safe_.state_.WrWM();
        rovio_node.mpFilter_->init_.state_.qWM() = rovio_node.mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
    if(finished)
      break;
  }

  //rovio_node.reset();

  bagIn.close();
  trace_est_pose.close();
  trace_timing.close();

  return 0;
}
