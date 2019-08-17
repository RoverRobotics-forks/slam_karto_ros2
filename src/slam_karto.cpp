/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/

#include "rclcpp/rclcpp.hpp"
//#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "open_karto/Mapper.h"

#include "spa_solver.h"

#include <boost/thread.hpp>

#include <map>
#include <string>
#include <vector>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
public:
  SlamKarto();
  ~SlamKarto();

  void laserCallback(const sensor_msgs::msg::LaserScan::ConstPtr & scan);
  bool mapCallback(nav_msgs::srv::GetMap::Request & req, nav_msgs::srv::GetMap::Response & res);
  void spin() { rclcpp::spin(node_.get_node_base_interface()); };

private:
  bool getOdomPose(karto::Pose2 & karto_pose, const rclcpp::Time & t);
  karto::LaserRangeFinder * getLaser(const sensor_msgs::msg::LaserScan::ConstPtr & scan);
  bool addScan(
    karto::LaserRangeFinder * laser, const sensor_msgs::msg::LaserScan::ConstPtr & scan,
    karto::Pose2 & karto_pose);
  bool updateMap();
  void publishTransform();
  void publishLoop(double transform_publish_period);
  void publishGraphVisualization();

  // ROS handles
  rclcpp::Node node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  tf2_ros::TransformBroadcaster * tfB_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr ss_;

  // The map that will be published / send to service callers
  nav_msgs::srv::GetMap::Response map_;

  // Storage for ROS parameters
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  int throttle_scans_;
  rclcpp::Duration map_update_interval_{0};
  double resolution_;
  boost::mutex map_mutex_;
  boost::mutex map_to_odom_mutex_;

  // Karto bookkeeping
  std::unique_ptr<karto::Mapper> mapper_;
  std::unique_ptr<karto::Dataset> dataset_;
  std::unique_ptr<SpaSolver> solver_;
  std::map<std::string, karto::LaserRangeFinder *> lasers_;
  std::map<std::string, bool> lasers_inverted_;

  // Internal state
  bool got_map_;
  int laser_count_;
  boost::thread * transform_thread_;
  tf2::Transform map_to_odom_;
  unsigned marker_count_;
  bool inverted_laser_;
};

SlamKarto::SlamKarto()
: node_{"slam_karto"},
  got_map_(false),
  laser_count_(0),
  transform_thread_(nullptr),
  marker_count_(0)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters

  odom_frame_ = node_.declare_parameter("odom_frame", "odom");
  map_frame_ = node_.declare_parameter("map_frame", "map");
  base_frame_ = node_.declare_parameter("base_frame", "base_link");
  throttle_scans_ = node_.declare_parameter("throttle_scans", 1);
  map_update_interval_ =
    rclcpp::Duration::from_seconds(node_.declare_parameter("map_update_interval", 5.0));

  if (!node_.get_parameter("resolution", resolution_)) {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    node_.get_parameter_or("delta", resolution_, 0.05);
  }

  double transform_publish_period = node_.declare_parameter("transform-publish_period", 0.05);

  // Set up publishers and subscriptions
  tfB_ = new tf2_ros::TransformBroadcaster(node_);
  sst_ =
    node_.create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
  sstm_ = node_.create_publisher<nav_msgs::msg::MapMetaData>(
    "map_metadata", rclcpp::QoS(1).transient_local());
  ss_ = node_.create_service<nav_msgs::srv::GetMap>(
    "dynamic_map", [this](auto x, auto y) { mapCallback(x, y); });
  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    node_, "scan", rclcpp::SensorDataQoS());
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback([this](auto & c) { laserCallback(c); });
  // boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ =
    node_.create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ =
    new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Initialize Karto structures
  mapper_ = std::make_unique<karto::Mapper>();
  dataset_ = std::make_unique<karto::Dataset>();

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if (node_.get_parameter("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter;
  if (node_.get_parameter("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance;
  if (node_.get_parameter("minimum_travel_distance", minimum_travel_distance))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading;
  if (node_.get_parameter("minimum_travel_heading", minimum_travel_heading))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size;
  if (node_.get_parameter("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if (node_.get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if (node_.get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if (node_.get_parameter("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if (node_.get_parameter("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if (node_.get_parameter("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if (node_.get_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if (node_.get_parameter("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if (node_.get_parameter("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if (node_.get_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if (node_.get_parameter("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if (node_.get_parameter(
        "correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if (node_.get_parameter(
        "correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

  double loop_search_space_dimension;
  if (node_.get_parameter("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if (node_.get_parameter("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if (node_.get_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if (node_.get_parameter("distance_variance_penalty", distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if (node_.get_parameter("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if (node_.get_parameter("fine_search_angle_offset", fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if (node_.get_parameter("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if (node_.get_parameter("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if (node_.get_parameter("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if (node_.get_parameter("minimum_distance_penalty", minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if (node_.get_parameter("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);

  // Set solver to be used in loop closure
  solver_ = std::make_unique<SpaSolver>();
  mapper_->SetScanSolver(solver_.get());  // todo: this should be a shared_ptr
}

SlamKarto::~SlamKarto()
{
  if (transform_thread_) {
    transform_thread_->join();
    delete transform_thread_;
  }
  scan_filter_.reset();
  scan_filter_sub_.reset();
  solver_.reset();
  solver_.reset();
  mapper_.reset();
  dataset_.release();
  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

void SlamKarto::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0) return;

  rclcpp::Rate r(1.0 / transform_publish_period);
  while (rclcpp::ok()) {
    publishTransform();
    r.sleep();
  }
}

void SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  rclcpp::Time tf_expiration = node_.now() + rclcpp::Duration::from_seconds(.05);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = node_.now();
  msg.child_frame_id = odom_frame_;
  tf2::fromMsg(map_to_odom_, msg.transform);
  tfB_->sendTransform(msg);
}

karto::LaserRangeFinder * SlamKarto::getLaser(const sensor_msgs::msg::LaserScan::ConstPtr & scan)
{
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try {

      tf_.transformPose(base_frame_, ident, laser_pose);
    } catch (tf::TransformException e) {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
      return nullptr;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    RCLCPP_INFO(node_.get_logger(),
      "laser %s's pose wrt base: %.3f %.3f %.3f", scan->header.frame_id.c_str(),
      laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    } catch (tf::TransformException & e) {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return nullptr;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse) ROS_INFO("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder * laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    // laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool SlamKarto::getOdomPose(karto::Pose2 & karto_pose, const rclcpp::Time & t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(
    tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  } catch (tf::TransformException e) {
    RCLCPP_WARN(node_.get_logger(),"Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = karto::Pose2(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);
  return true;
}

void SlamKarto::publishGraphVisualization()
{
  std::vector<float> graph;
  solver_->getGraph(graph);

  visualization_msgs::msg::MarkerArray marray;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = node_.now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = rclcpp::Duration(0);

  visualization_msgs::msg::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = node_.now();
  edge.action = visualization_msgs::msg::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::msg::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::msg::Marker::ADD;
  uint id = 0;
  for (uint i = 0; i < graph.size() / 2; i++) {
    m.id = id;
    m.pose.position.x = graph[2 * i];
    m.pose.position.y = graph[2 * i + 1];
    marray.markers.push_back(visualization_msgs::msg::Marker(m));
    id++;

    if (i > 0) {
      edge.points.clear();

      geometry_msgs::msg::Point p;
      p.x = graph[2 * (i - 1)];
      p.y = graph[2 * (i - 1) + 1];
      edge.points.push_back(p);
      p.x = graph[2 * i];
      p.y = graph[2 * i + 1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::msg::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < marker_count_; id++) {
    m.id = id;
    marray.markers.push_back(visualization_msgs::msg::Marker(m));
  }

  marker_count_ = marray.markers.size();

  marker_publisher_->publish(marray);
}

void SlamKarto::laserCallback(const sensor_msgs::msg::LaserScan::ConstPtr & scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0) return;

  static rclcpp::Time last_map_update = scan->header.stamp;

  // Check whether we know about this laser yet
  karto::LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(
      node_.get_logger(), "Failed to create laser device for %s; discarding scan",
      scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if (addScan(laser, scan, odom_pose)) {
    RCLCPP_DEBUG(
      node_.get_logger(), "added scan at pose: %.3f %.3f %.3f", odom_pose.GetX(), odom_pose.GetY(),
      odom_pose.GetHeading());

    publishGraphVisualization();

    if (!got_map_ || (rclcpp::Time(scan->header.stamp) - last_map_update) > map_update_interval_) {
      if (updateMap()) {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        RCLCPP_DEBUG(node_.get_logger(), "Updated the map");
      }
    }
  }
}

bool SlamKarto::updateMap()
{
  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid * occ_grid =
    karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if (!occ_grid) return false;

  if (!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if (
    map_.map.info.width != (unsigned int)width || map_.map.info.height != (unsigned int)height ||
    map_.map.info.origin.position.x != offset.GetX() ||
    map_.map.info.origin.position.y != offset.GetY()) {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          RCLCPP_WARN(node_.get_logger(), "Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = node_.now();
  map_.map.header.frame_id = map_frame_;

  sst_->publish(map_.map);
  sstm_->publish(map_.map.info);

  delete occ_grid;

  return true;
}

bool SlamKarto::addScan(
  karto::LaserRangeFinder * laser, const sensor_msgs::msg::LaserScan::ConstPtr & scan,
  karto::Pose2 & karto_pose)
{
  if (!getOdomPose(karto_pose, scan->header.stamp)) return false;

  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for (std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
         it != scan->ranges.rend(); ++it) {
      readings.push_back(*it);
    }
  } else {
    for (std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end();
         ++it) {
      readings.push_back(*it);
    }
  }

  // create localized range scan
  auto * range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan))) {
    // std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " <<
    // range_scan->GetCorrectedPose() << std::endl;

    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try {
      tf_.transformPose(
        odom_frame_,
        tf::Stamped<tf::Pose>(
          tf::Transform(
            tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
            tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0))
            .inverse(),
          scan->header.stamp, base_frame_),
        odom_to_map);
    } catch (tf::TransformException e) {
      RCLCPP_ERROR(node_.get_logger(), "Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()))
        .inverse();
    map_to_odom_mutex_.unlock();

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  } else {
    delete range_scan;
  }
  return processed;
}

bool SlamKarto::mapCallback(
  nav_msgs::srv::GetMap::Request & req, nav_msgs::srv::GetMap::Response & res)
{
  boost::mutex::scoped_lock lock(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height) {
    res = map_;
    return true;
  } else
    return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  SlamKarto kn;
  kn.spin();

  return 0;
}
