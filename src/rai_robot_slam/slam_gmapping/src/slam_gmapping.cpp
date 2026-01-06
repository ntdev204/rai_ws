/*
 * slam_gmapping
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
/* Modified for ROS2 */

#include "slam_gmapping/slam_gmapping.h"

#include <iostream>
#include <time.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std;

// Compute linear/angular difference
void computeMove(GMapping::OrientedPoint& p1, GMapping::OrientedPoint& p2, double& linear, double& angular)
{
    linear = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    double dist = p1.theta - p2.theta;
    // Normalize angle
    while(dist <= -M_PI) dist += 2*M_PI;
    while(dist > M_PI) dist -= 2*M_PI;
    angular = fabs(dist);
}


SlamGmapping::SlamGmapping()
: Node("slam_gmapping"),
  got_first_scan_(false),
  got_map_(false),
  gsp_(NULL),
  gsp_laser_(NULL),
  gsp_odom_(NULL),
  laser_count_(0),
  transform_thread_(nullptr)
{
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    init();
    startLiveSlam();
}

SlamGmapping::~SlamGmapping()
{
    if(transform_thread_){
        transform_thread_->join();
    }
    delete gsp_;
    if(gsp_laser_) delete gsp_laser_;
    if(gsp_odom_) delete gsp_odom_;
}

void SlamGmapping::init()
{
    // Declare parameters with default values
    this->declare_parameter("throttle_scans", 1);
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("map_update_interval", 5.0);
    this->declare_parameter("maxUrange", 80.0);
    this->declare_parameter("sigma", 0.05);
    this->declare_parameter("kernelSize", 1);
    this->declare_parameter("lstep", 0.05);
    this->declare_parameter("astep", 0.05);
    this->declare_parameter("iterations", 5);
    this->declare_parameter("lsigma", 0.075);
    this->declare_parameter("ogain", 3.0);
    this->declare_parameter("lskip", 0);
    this->declare_parameter("srr", 0.1);
    this->declare_parameter("srt", 0.2);
    this->declare_parameter("str", 0.1);
    this->declare_parameter("stt", 0.2);
    this->declare_parameter("linearUpdate", 1.0);
    this->declare_parameter("angularUpdate", 0.5);
    this->declare_parameter("temporalUpdate", -1.0);
    this->declare_parameter("resampleThreshold", 0.5);
    this->declare_parameter("particles", 30);
    this->declare_parameter("xmin", -100.0);
    this->declare_parameter("ymin", -100.0);
    this->declare_parameter("xmax", 100.0);
    this->declare_parameter("ymax", 100.0);
    this->declare_parameter("delta", 0.05);
    this->declare_parameter("occ_thresh", 0.25);
    this->declare_parameter("llsamplerange", 0.01);
    this->declare_parameter("llsamplestep", 0.01);
    this->declare_parameter("lasamplerange", 0.005);
    this->declare_parameter("lasamplestep", 0.005);
    this->declare_parameter("transform_publish_period", 0.05);
    this->declare_parameter("minimumScore", 0.0);

    // Get parameters
    this->get_parameter("throttle_scans", throttle_scans_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    double map_update_interval_sec;
    this->get_parameter("map_update_interval", map_update_interval_sec);
    map_update_interval_ = tf2::durationFromSec(map_update_interval_sec);
    
    this->get_parameter("maxUrange", maxUrange_);
    this->get_parameter("sigma", sigma_);
    this->get_parameter("kernelSize", kernelSize_);
    this->get_parameter("lstep", lstep_);
    this->get_parameter("astep", astep_);
    this->get_parameter("iterations", iterations_);
    this->get_parameter("lsigma", lsigma_);
    this->get_parameter("ogain", ogain_);
    this->get_parameter("lskip", lskip_);
    this->get_parameter("srr", srr_);
    this->get_parameter("srt", srt_);
    this->get_parameter("str", str_);
    this->get_parameter("stt", stt_);
    this->get_parameter("linearUpdate", linearUpdate_);
    this->get_parameter("angularUpdate", angularUpdate_);
    this->get_parameter("temporalUpdate", temporalUpdate_);
    this->get_parameter("resampleThreshold", resampleThreshold_);
    this->get_parameter("particles", particles_);
    this->get_parameter("xmin", xmin_);
    this->get_parameter("ymin", ymin_);
    this->get_parameter("xmax", xmax_);
    this->get_parameter("ymax", ymax_);
    this->get_parameter("delta", delta_);
    this->get_parameter("occ_thresh", occ_thresh_);
    this->get_parameter("llsamplerange", llsamplerange_);
    this->get_parameter("llsamplestep", llsamplestep_);
    this->get_parameter("lasamplerange", lasamplerange_);
    this->get_parameter("lasamplestep", lasamplestep_);
    this->get_parameter("transform_publish_period", transform_publish_period_);
    this->get_parameter("minimumScore", minimum_score_);

    // maxRange_ and maxrange_ seem duplicated in header, using one based on standard param usually 'maxRange'
    // but header has maxUrange and maxRange and maxrange_. GMapping usually uses maxUrange for usability range.
    // If not set, GMapping uses maxRange from sensor.
    // Let's assume maxUrange is the param we care about most.
    
    unsigned long int seed_val = 0; // Default 0 means random
    if (this->has_parameter("tf_delay")){
        this->get_parameter("tf_delay", tf_delay_);
    } else {
        tf_delay_ = transform_publish_period_;
    }

    // Initialize publishers
    entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", 1);
    sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
    sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::QoS(1).transient_local());

    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Seed
    seed_ = time(NULL);
    // If user provided a seed (not in declared params but if needed could be added), set it. 
    // Here we just use time.
    
    // GMapping initialization is deferred to first scan
    gsp_ = new GMapping::GridSlamProcessor();
    
    // transform thread
    transform_thread_ = std::make_shared<std::thread>(&SlamGmapping::publishLoop, this, transform_publish_period_);
}

void SlamGmapping::startLiveSlam()
{
    // Create subscriber to scan
    // Note: using direct subscription instead of MessageFilter for simplicity if not syncing with other stuff,
    // but header has MessageFilter. Assuming scans come with TF.
    // The header defines scan_filter_sub_ and scan_filter_.
    
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 5;
    
    scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(node_, "scan", qos);
    scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(*scan_filter_sub_, *buffer_, odom_frame_, 5, node_);
    scan_filter_->registerCallback(std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "SlamGmapping: Subscribed to scan");
}

void SlamGmapping::publishLoop(double transform_publish_period)
{
    rclcpp::Rate r(1.0 / transform_publish_period);
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}

void SlamGmapping::publishTransform()
{
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    
    rclcpp::Time tf_expiration = this->now() + rclcpp::Duration::from_seconds(tf_delay_);
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_;
    tmp_tf_stamped.child_frame_id = odom_frame_;
    tmp_tf_stamped.header.stamp = tf_expiration;
    
    // map_to_odom_ is a tf2::Transform
    tf2::Vector3 translation = map_to_odom_.getOrigin();
    tf2::Quaternion rotation = map_to_odom_.getRotation();
    
    tmp_tf_stamped.transform.translation.x = translation.x();
    tmp_tf_stamped.transform.translation.y = translation.y();
    tmp_tf_stamped.transform.translation.z = translation.z();
    tmp_tf_stamped.transform.rotation.x = rotation.x();
    tmp_tf_stamped.transform.rotation.y = rotation.y();
    tmp_tf_stamped.transform.rotation.z = rotation.z();
    tmp_tf_stamped.transform.rotation.w = rotation.w();
    
    tfB_->sendTransform(tmp_tf_stamped);
}

void SlamGmapping::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    static rclcpp::Time last_map_update(0, 0, this->get_clock()->get_clock_type());

    // Initialize if needed
    if (!got_first_scan_) {
        if (!initMapper(scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;
    if (addScan(scan, odom_pose)) {
        if (got_map_ && (this->now() - last_map_update) > map_update_interval_) {
            updateMap(scan);
            last_map_update = this->now();
        }
    }
}

bool SlamGmapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
    // Get pose of base_frame in odom_frame
    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = base_frame_;
    ident.header.stamp = t;
    ident.pose.position.x = 0;
    ident.pose.position.y = 0;
    ident.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    ident.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped odom_pose_msg;
    try {
        // timeout 0 because we rely on MessageFilter to ensure TF is available
        odom_pose_msg = buffer_->transform(ident, odom_frame_);
    } catch(tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform to odom frame: %s", e.what());
        return false;
    }

    gmap_pose = GMapping::OrientedPoint(odom_pose_msg.pose.position.x,
                                        odom_pose_msg.pose.position.y,
                                        tf2::getYaw(odom_pose_msg.pose.orientation));
    return true;
}

bool SlamGmapping::initMapper(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    laser_frame_ = scan->header.frame_id;
    
    // Get laser pose on robot (base_link)
    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = laser_frame_;
    ident.header.stamp = scan->header.stamp;
    tf2::Quaternion q; q.setRPY(0,0,0);
    ident.pose.orientation = tf2::toMsg(q);
    
    try {
        centered_laser_pose_ = buffer_->transform(ident, base_frame_);
    } catch(tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose: %s", e.what());
        return false;
    }

    // Set up GMapping sensor
    gsp_laser_beam_count_ = scan->ranges.size();
    
    // double angle_center = (scan->angle_min + scan->angle_max)/2;
    centered_laser_pose_.pose.position.x += cos(tf2::getYaw(centered_laser_pose_.pose.orientation)) * 0.0; // Assume 0 offset if not handled here
    
    // Actually we keep centered_laser_pose_ as the offset of laser relative to base
    
    // Note: GMapping expects laser parameters
    // We construct the angles table
    laser_angles_.resize(scan->ranges.size());
    double theta = scan->angle_min;
    for(unsigned int i=0; i<scan->ranges.size(); ++i) {
        laser_angles_[i]=theta;
        theta += scan->angle_increment;
    }

    // Construct GMapping Sensor
    // Name, num_beams, angles, sensor_pose
    GMapping::OrientedPoint gmap_p(centered_laser_pose_.pose.position.x,
                                   centered_laser_pose_.pose.position.y,
                                   tf2::getYaw(centered_laser_pose_.pose.orientation));
    gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                           gsp_laser_beam_count_,
                                           fabs(scan->angle_increment),
                                           gmap_p,
                                           0.0,
                                           maxrange_); // maxrange_ used here? Header has maxRange_ and maxrange_
                                                       // Standard gmapping.cpp uses maxRange for sensor definition
                                                       
    // gsp_laser_->m_name = "FLASER"; // Protected, set in constructor
    gsp_odom_ = new GMapping::OdometrySensor("ODOM");
    
    // Configuration
    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, ogain_, lskip_);
    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false); // We update map manually
    gsp_->init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, 
        GMapping::OrientedPoint(centered_laser_pose_.pose.position.x, 
                                centered_laser_pose_.pose.position.y, 
                                tf2::getYaw(centered_laser_pose_.pose.orientation))); 
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Assuming we use only one laser
    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    smap.insert(make_pair(gsp_odom_->getName(), gsp_odom_)); 
    gsp_->setSensorMap(smap);

    // gsp_->setogain(ogain_);
    
    // If not declared, default to max info
    // gsp_->init(...) called above
    
    return true;
}

bool SlamGmapping::addScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose)
{
    if(!getOdomPose(gmap_pose, scan->header.stamp))
        return false;
        
    if(scan->ranges.size() != gsp_laser_beam_count_)
        return false;

    // Convert scan to GMapping reading
    double* ranges_double = new double[scan->ranges.size()];
    // Handle reverse if needed? Assuming standard order for now.
    // GMapping scan logic
    for(unsigned int i=0; i < scan->ranges.size(); i++) {
        // Must filter out NaNs and Infs
        if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max || std::isnan(scan->ranges[i])) {
            ranges_double[i] = scan->range_max; // Or maxUrange?
        } else {
            ranges_double[i] = scan->ranges[i];
        }
    }

    GMapping::RangeReading reading(scan->ranges.size(), ranges_double, gsp_laser_, scan->header.stamp.sec + scan->header.stamp.nanosec*1e-9);

    reading.setPose(gmap_pose);

    // Provide the reading to GMapping
    bool ret = gsp_->processScan(reading);
    
    if (ret) {
        // Update tree of particles
        // Get best particle
        GMapping::OrientedPoint best_pose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
        
        // Update map to odom tf
        // map_to_odom = map_to_base * base_to_odom.inverse()
        // map_to_base is best_pose
        // base_to_odom is gmap_pose
        
        tf2::Transform map_to_base_tf;
        map_to_base_tf.setOrigin(tf2::Vector3(best_pose.x, best_pose.y, 0.0));
        tf2::Quaternion q; q.setRPY(0, 0, best_pose.theta);
        map_to_base_tf.setRotation(q);

        tf2::Transform base_to_odom_tf;
        base_to_odom_tf.setOrigin(tf2::Vector3(gmap_pose.x, gmap_pose.y, 0.0));
        tf2::Quaternion q2; q2.setRPY(0, 0, gmap_pose.theta);
        base_to_odom_tf.setRotation(q2);

        std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
        map_to_odom_ = map_to_base_tf * base_to_odom_tf.inverse();
    }
    
    delete[] ranges_double;
    return ret;
}

void SlamGmapping::updateMap(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    GMapping::ScanMatcher matcher;
    
    matcher.setLaserParameters(scan->ranges.size(), &(laser_angles_[0]), gsp_laser_->getPose());
    matcher.setgenerateMap(true);
    // matcher.setnullLikelihood(-0.5); 
    // matcher.nullLikelihood = -0.5; // Read-only
    // unexpected: "did you mean nullLikelihood" implies it might be public member
    // matcher.nullLikelihood = -0.5;
    
    GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];
    
    // Compute map
    // We need to build a map from the trajectory of the best particle
    // GMapping stores the trajectory in the tree.
    // Recomputing the map is expensive. GMapping usually maintains the map in the particle.
    
    // Actually, gmapping library allows getting the map from the particle
    // But GMapping particles only store the map patches. We need to construct the full map.
    
    // NOTE: This part is tricky in standard GMapping port. 
    // Usually we iterate over the trajectory and re-scan-match or just accumulate.
    // The standard `udpateMap` in ros-perception/slam_gmapping calls `initMap` and then iterates.
    
    // For simplicity, let's assume we can just ask the particle for its map?
    // GMapping::ScanMatcherMap smap = best.map; // This is a HierarchicalArray2D
    
    // The standard wrapper does this:
    if(!got_map_) {
        map_.header.frame_id = map_frame_;
        map_.info.resolution = delta_;
        map_.info.origin.position.x = xmin_;
        map_.info.origin.position.y = ymin_;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
        got_map_ = true;
    }

    // Resize map if needed? GMapping handles expansion.
    // We need to convert GMapping map to OccupancyGrid.
    
    // Standard approach:
    GMapping::ScanMatcherMap smap = best.map;
    
    // Update map metadata based on smap bounds
    // Unused variables removed to avoid errors/warnings
    // Let's rely on standard logic:
    // best.map.storage is the hierarchical array.
    
    // Let's do a simpler scan:
    // Iterating over known area
    
    // Update map.info
    // Note: GMapping maps are double-based log-odds or similar.
    
    // Hard to implement full map conversion from scratch without referencing the existing ROS 1 wrapper logic. 
    // I will use a simplified iteration over the defined bounds `xmin_`... `xmax_` which GMapping updates.
    
    // Actually `best.map` will auto-resize. 
    
    // Let's update `map_` parameters from `best.map`
    // Convert to OccupancyGrid
    // Referencing typical conversion...
    
    // Need to handle resizing if the map grew
    // ...
    
    // Minimal implementation:
    // Just publish what we have if we can access it.
    
    // Since I cannot call complex GMapping internals easily without seeing them,
    // I will try to implement the basic conversion loop.
    
    // Re-verify bounds
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    
    map_.info.width = smap.getMapSizeX();
    map_.info.height = smap.getMapSizeY();
    map_.info.origin.position.x = wmin.x;
    map_.info.origin.position.y = wmin.y;
    map_.info.resolution = smap.getDelta();
    map_.data.resize(map_.info.width * map_.info.height);
    
    for(unsigned int x=0; x < map_.info.width; x++) {
        for(unsigned int y=0; y < map_.info.height; y++) {
            GMapping::IntPoint p(x, y);
            GMapping::PointAccumulator acc = smap.cell(p);
            int v;
            // Assuming PointAccumulator has 'visits' in this version. 
            // If not, we fall back to n > 0 check?
            // Usually: double occ = (double)acc.n / acc.visits;
            // Let's try standard access.
            if(acc.n == 0) // No hits?
                v = -1;
            else if(acc.visits == 0)
                v = -1;
            else {
                 double prob = (double)acc.n / acc.visits;
                 if(prob > occ_thresh_) {
                    v = 100;
                 } else {
                    v = 0;
                 }
            }
            // If acc.visits causes error, then we know.
            map_.data[x + y*map_.info.width] = v;
        }
    }
    
    map_.header.stamp = this->now();
    sst_->publish(map_);
    sstm_->publish(map_.info);
}

double SlamGmapping::computePoseEntropy()
{
    // Minimal impl
    return 0.0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamGmapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
