/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/DSElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// Grid Map
#include <grid_map_msgs/GridMap.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

using namespace std;
using namespace grid_map;

namespace elevation_mapping {

DSElevationMap::DSElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      staticMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      dynamicMap_({"error", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      finalMap_({"elevation", "variance"}),
      preMap_({"elevation", "variance", "color", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan", "error_static", "to_dyn_or_sta"}),
      hasUnderlyingMap_(false),
      visibilityCleanupDuration_(0.0)
{
    readParam();
  staticMap_.setBasicLayers({"elevation", "variance"});
  dynamicMap_.setBasicLayers({"error", "variance"});
  preMap_.setBasicLayers({"elevation", "variance"});
  finalMap_.setBasicLayers({"elevation", "variance"});
  clear();

  elevationMapStaticPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_static", 1);
  elevationMapDynamicPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_dynamic", 1);
  elevationMapFinalPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_final", 1);
  elevationMapPresentPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_present", 1);
//  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
//      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
  // TODO if (enableVisibilityCleanup_) when parameter cleanup is ready.
//  visbilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

  initialTime_ = ros::Time::now();
}

DSElevationMap::~DSElevationMap()
{
}

void DSElevationMap::readParam()
{
    nodeHandle_.param("error_measure_threshold", error_measure_threshold, 0.2);
    nodeHandle_.param("dynamic_variance_threshold", dynamic_variance_threshold, 0.1);
    nodeHandle_.param("max_rate_error_grids", max_rate_error_grids, 0.4);
    nodeHandle_.param("dtos_threshold", dtos_threshold, 0.05);
    nodeHandle_.param("obj_speed_factor", obj_speed_factor, 0.5);

    sensorProcessor_.reset(new LaserSensorProcessorSimple(nodeHandle_, transformListener_));
    //sensorProcessor_.readParameters();
    ROS_INFO("dsmap read param");
}

void DSElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position)
{
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  staticMap_.setGeometry(length, resolution, position);
  dynamicMap_.setGeometry(length, resolution, position);
  preMap_.setGeometry(length, resolution, position);
  finalMap_.setGeometry(length, resolution, position);
  length_grid_num = length(0) / resolution;
  ROS_INFO_STREAM("Elevation map grid resized to " << staticMap_.getSize()(0) << " rows and "  << staticMap_.getSize()(1) << " columns.");
}

bool DSElevationMap::ds_newadd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
               const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
               const ros::Time& timeStamp, const Eigen::Affine3d& transformationSensorToMap)
{
    // Initialization for time calculation.
    const ros::WallTime methodStartTime(ros::WallTime::now());
    int64_t begin = clock(), end;

    // First filter, like original process() but without computevariance
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudProcessed(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensorProcessor_->preprocess(pointCloud, pointCloudProcessed);

    // Then do the first part of computevariance
    sensorProcessor_->precompute(robotPoseCovariance);
    end = clock();
    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    cout<<"pre compute time: "<<search_time<<"s."<<endl;

    // Here start the original add part
    ROS_INFO("start add points");

    // Update initial time if it is not initialized.
    if (initialTime_.toSec() == 0) {
      initialTime_ = timeStamp;
    }
    const double scanTimeSinceInitialization = (timeStamp - initialTime_).toSec();

    // before adding point cloud clear present grid
    preMap_.clearBasic();
    int numof_total_grid_to_update = 0;
    Eigen::MatrixXi todors(length_grid_num, length_grid_num);
    todors.setZero();

//    double time_step0=0, time_step1=0, time_step2=0, time_step3=0;
    int64_t step_st = clock(), step_et;

    for (unsigned int i = 0; i < pointCloudProcessed->size(); ++i) {
        auto& point = pointCloudProcessed->points[i];
        auto& point_sensor = pointCloud->points[i];
        Index index;
        Position position(point.x, point.y);
        // Skip this point if it does not lie within the elevation map.
        if (!preMap_.getIndex(position, index)) continue;

        auto& s_elevation = staticMap_.at("elevation", index);
        auto& s_variance = staticMap_.at("variance", index);
//        auto& s_lowestScanPoint = staticMap_.at("lowest_scan_point", index);
//        auto& s_sensorXatLowestScan = staticMap_.at("sensor_x_at_lowest_scan", index);
//        auto& s_sensorYatLowestScan = staticMap_.at("sensor_y_at_lowest_scan", index);
//        auto& s_sensorZatLowestScan = staticMap_.at("sensor_z_at_lowest_scan", index);

        //auto& d_error = dynamicMap_.at("error", index);
        auto& d_variance = dynamicMap_.at("variance", index);
//        auto& d_lowestScanPoint = dynamicMap_.at("lowest_scan_point", index);
//        auto& d_sensorXatLowestScan = dynamicMap_.at("sensor_x_at_lowest_scan", index);
//        auto& d_sensorYatLowestScan = dynamicMap_.at("sensor_y_at_lowest_scan", index);
//        auto& d_sensorZatLowestScan = dynamicMap_.at("sensor_z_at_lowest_scan", index);

        auto& p_elevation = preMap_.at("elevation", index);
        auto& p_variance = preMap_.at("variance", index);
        auto& p_lowestScanPoint = preMap_.at("lowest_scan_point", index);
        auto& p_sensorXatLowestScan = preMap_.at("sensor_x_at_lowest_scan", index);
        auto& p_sensorYatLowestScan = preMap_.at("sensor_y_at_lowest_scan", index);
        auto& p_sensorZatLowestScan = preMap_.at("sensor_z_at_lowest_scan", index);
        auto& p_error = preMap_.at("error_static", index);
        auto& p_to_dyn_or_sta = preMap_.at("to_dyn_or_sta", index);

        //calculate variance here
        const float& pointVariance = sensorProcessor_->computeVarOfOnePoint(point_sensor);

        // add to present map
        if (!preMap_.isValid(index)) {
            // No prior information in present map, use measurement.
            p_elevation = point.z;
            p_variance = pointVariance;

            // Store lowest points from scan for visibility checking.
            const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
            p_lowestScanPoint = pointHeightPlusUncertainty;
            const Position3 sensorTranslation(transformationSensorToMap.translation());
            p_sensorXatLowestScan = sensorTranslation.x();
            p_sensorYatLowestScan = sensorTranslation.y();
            p_sensorZatLowestScan = sensorTranslation.z();

            ++numof_total_grid_to_update;
        }
        else{
            const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
            if(p_elevation < point.z){
                p_elevation = point.z;
                p_variance = pointVariance;
            }
            else if(pointHeightPlusUncertainty < p_lowestScanPoint){
                p_lowestScanPoint = pointHeightPlusUncertainty;
            }
        }

        //check if error over threshold, decide update to static or dynamic
        if(staticMap_.isValid(index) && s_variance > 0){
            p_error = p_elevation - s_elevation;
            if(fabs(p_error) > error_measure_threshold || p_variance > dynamic_variance_threshold){
                p_to_dyn_or_sta = 1;   // to dynamic
                todors(index(0), index(1)) = 1;
                p_variance += obj_speed_factor;
            }
            else{
                if(dynamicMap_.isValid(index) && d_variance > 0){
                    p_to_dyn_or_sta = 1;   // to dynamic
                    todors(index(0), index(1)) = 1;
                    p_variance += obj_speed_factor;
                }
                else{
                    p_to_dyn_or_sta = 0;   // to static
                    todors(index(0), index(1)) = 0;
                }
            }
        }
        else{
            s_elevation = p_elevation;
            s_variance = p_variance;
            p_error = 0;
            p_to_dyn_or_sta = 1;   // to dynamic
            todors(index(0), index(1)) = 1;
            p_variance += obj_speed_factor;
        }
    }
    step_et = clock();
    double time_loop = ((double)(step_et - step_st) / CLOCKS_PER_SEC);
    cout<<"loop time: "<<time_loop<<"s."<<endl;

    // check whether the observation is ok to add
    cout<<"error grids: "<<todors.sum()<<", total observe grids: "
       <<numof_total_grid_to_update<<", points: "<<pointCloudProcessed->size()<<endl;
    if(true){//1.0*numof_error_grids/numof_total_grid_to_update < max_rate_error_grids){
        // add data from preMap to dynamic or static map, from dynamic to static map
        fusePreToStaDyn();
    }
    else{
        // abandon this observation
        // OR(TODO) refine this observation
    }

    // add dynamic and static to final map.
    end = clock();
    search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    const ros::WallDuration duration = ros::WallTime::now() - methodStartTime;
    ROS_INFO("Raw map has been updated with a new point cloud in %f s(ros)|| %f s(sys).", duration.toSec(), search_time);
    return true;
}


bool DSElevationMap::ds_add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf &pointCloudVariances, const ros::Time &timeStamp, const Eigen::Affine3d &transformationSensorToMap)
{
    if (pointCloud->size() != pointCloudVariances.size()) {
      ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
                (int) pointCloud->size(), (int) pointCloudVariances.size());
      return false;
    }
    ROS_INFO("start add points");
    int64_t begin = clock(), end;
    // Initialization for time calculation.
    const ros::WallTime methodStartTime(ros::WallTime::now());

    // Update initial time if it is not initialized.
    if (initialTime_.toSec() == 0) {
      initialTime_ = timeStamp;
    }
    const double scanTimeSinceInitialization = (timeStamp - initialTime_).toSec();

    // before adding point cloud clear present grid
    preMap_.clearBasic();
    int numof_total_grid_to_update = 0, numof_error_grids = 0;
    Eigen::MatrixXi todors(length_grid_num, length_grid_num);
    todors.setZero();
    for (unsigned int i = 0; i < pointCloud->size(); ++i) {
        auto& point = pointCloud->points[i];
        Index index;
        Position position(point.x, point.y);
        // Skip this point if it does not lie within the elevation map.
        if (!preMap_.getIndex(position, index)) continue;

        auto& s_elevation = staticMap_.at("elevation", index);
        auto& s_variance = staticMap_.at("variance", index);
        auto& s_lowestScanPoint = staticMap_.at("lowest_scan_point", index);
        auto& s_sensorXatLowestScan = staticMap_.at("sensor_x_at_lowest_scan", index);
        auto& s_sensorYatLowestScan = staticMap_.at("sensor_y_at_lowest_scan", index);
        auto& s_sensorZatLowestScan = staticMap_.at("sensor_z_at_lowest_scan", index);

        auto& d_error = dynamicMap_.at("error", index);
        auto& d_variance = dynamicMap_.at("variance", index);
        auto& d_lowestScanPoint = dynamicMap_.at("lowest_scan_point", index);
        auto& d_sensorXatLowestScan = dynamicMap_.at("sensor_x_at_lowest_scan", index);
        auto& d_sensorYatLowestScan = dynamicMap_.at("sensor_y_at_lowest_scan", index);
        auto& d_sensorZatLowestScan = dynamicMap_.at("sensor_z_at_lowest_scan", index);

        auto& p_elevation = preMap_.at("elevation", index);
        auto& p_variance = preMap_.at("variance", index);
        auto& p_lowestScanPoint = preMap_.at("lowest_scan_point", index);
        auto& p_sensorXatLowestScan = preMap_.at("sensor_x_at_lowest_scan", index);
        auto& p_sensorYatLowestScan = preMap_.at("sensor_y_at_lowest_scan", index);
        auto& p_sensorZatLowestScan = preMap_.at("sensor_z_at_lowest_scan", index);
        auto& p_error = preMap_.at("error_static", index);
        auto& p_to_dyn_or_sta = preMap_.at("to_dyn_or_sta", index);

        const float& pointVariance = pointCloudVariances(i);

        // add to present map
        if (!preMap_.isValid(index)) {
            // No prior information in present map, use measurement.
            p_elevation = point.z;
            p_variance = pointVariance;

            // Store lowest points from scan for visibility checking.
            const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
            p_lowestScanPoint = pointHeightPlusUncertainty;
            const Position3 sensorTranslation(transformationSensorToMap.translation());
            p_sensorXatLowestScan = sensorTranslation.x();
            p_sensorYatLowestScan = sensorTranslation.y();
            p_sensorZatLowestScan = sensorTranslation.z();

            ++numof_total_grid_to_update;
        }
        else{
            const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
            if(p_elevation < point.z){
                p_elevation = point.z;
                p_variance = pointVariance;
            }
            else if(pointHeightPlusUncertainty < p_lowestScanPoint){
                p_lowestScanPoint = pointHeightPlusUncertainty;
            }
        }

        //check if error over threshold, decide update to static or dynamic
        if(staticMap_.isValid(index) && s_variance > 0){
            p_error = p_elevation - s_elevation;
            if(fabs(p_error) > error_measure_threshold || p_variance > dynamic_variance_threshold){
                p_to_dyn_or_sta = 1;   // to dynamic
                todors(index(0), index(1)) = 1;
                p_variance += obj_speed_factor;
            }
            else{
                if(dynamicMap_.isValid(index) && d_variance > 0){
                    p_to_dyn_or_sta = 1;   // to dynamic
                    todors(index(0), index(1)) = 1;
                    p_variance += obj_speed_factor;
                }
                else{
                    p_to_dyn_or_sta = 0;   // to static
                    todors(index(0), index(1)) = 0;
                }
            }
        }
        else{
            s_elevation = p_elevation;
            s_variance = p_variance;
            p_error = 0;
            p_to_dyn_or_sta = 1;   // to dynamic
            todors(index(0), index(1)) = 1;
            p_variance += obj_speed_factor;
        }
//        p_error = p_elevation - s_elevation;
//        if((staticMap_.isValid(index) && fabs(p_error) > error_measure_threshold) || p_variance > dynamic_variance_threshold){
//            ++numof_error_grids;
//            p_to_dyn_or_sta = 1;   // to dynamic
//        }
//        else{
//            p_to_dyn_or_sta = 0;   // to static
//        }
    }

    // check whether the observation is ok to add
    cout<<"error grids: "<<todors.sum()<<", total observe grids: "
       <<numof_total_grid_to_update<<", points: "<<pointCloud->size()<<endl;
    if(true){//1.0*numof_error_grids/numof_total_grid_to_update < max_rate_error_grids){
        // add data from preMap to dynamic or static map, from dynamic to static map
        fusePreToStaDyn();
    }
    else{
        // abandon this observation
        // OR(TODO) refine this observation
    }

    // add dynamic and static to final map.
    end = clock();
    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    const ros::WallDuration duration = ros::WallTime::now() - methodStartTime;
    ROS_INFO("Raw map has been updated with a new point cloud in %f s(ros)|| %f s(sys).", duration.toSec(), search_time);
    return true;
}

bool DSElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const ros::Time& timestamp, const Eigen::Affine3d& transformationSensorToMap)
{
  if (pointCloud->size() != pointCloudVariances.size()) {
    ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
              (int) pointCloud->size(), (int) pointCloudVariances.size());
    return false;
  }

  // Initialization for time calculation.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  // Update initial time if it is not initialized.
  if (initialTime_.toSec() == 0) {
    initialTime_ = timestamp;
  }
  const double scanTimeSinceInitialization = (timestamp - initialTime_).toSec();

  // ROS_INFO("add (%i) points to map", (int)pointCloud->size());
  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    auto& point = pointCloud->points[i];
    Index index;
    Position position(point.x, point.y);
    if (!staticMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

    auto& elevation = staticMap_.at("elevation", index);
    auto& variance = staticMap_.at("variance", index);
    auto& horizontalVarianceX = staticMap_.at("horizontal_variance_x", index);
    auto& horizontalVarianceY = staticMap_.at("horizontal_variance_y", index);
    auto& horizontalVarianceXY = staticMap_.at("horizontal_variance_xy", index);
    auto& color = staticMap_.at("color", index);
    auto& time = staticMap_.at("time", index);
    auto& lowestScanPoint = staticMap_.at("lowest_scan_point", index);
    auto& sensorXatLowestScan = staticMap_.at("sensor_x_at_lowest_scan", index);
    auto& sensorYatLowestScan = staticMap_.at("sensor_y_at_lowest_scan", index);
    auto& sensorZatLowestScan = staticMap_.at("sensor_z_at_lowest_scan", index);

    const float& pointVariance = pointCloudVariances(i);
    const float scanTimeSinceInitialization = (timestamp - initialTime_).toSec();

    if (!staticMap_.isValid(index)) {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    // Deal with multiple heights in one cell.
    const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);
    if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
      if (scanTimeSinceInitialization - time <= scanningDuration_ && elevation > point.z) {
        // Ignore point if measurement is from the same point cloud (time comparison) and
        // if measurement is lower then the elevation in the map.
      } else if (scanTimeSinceInitialization - time <= scanningDuration_) {
        // If point is higher.
        elevation = point.z;
        variance = pointVariance;
      } else {
        variance += multiHeightNoise_;
      }
      continue;
    }

    // Store lowest points from scan for visibility checking.
    const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance); // 3 sigma.
    if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint){
      lowestScanPoint = pointHeightPlusUncertainty;
      const Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
    }

    // Fuse measurement with elevation map data.
    elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
    variance = (pointVariance * variance) / (pointVariance + variance);
    // TODO Add color fusion.
    colorVectorToValue(point.getRGBVector3i(), color);
    time = scanTimeSinceInitialization;

    // Horizontal variances are reset.
    horizontalVarianceX = minHorizontalVariance_;
    horizontalVarianceY = minHorizontalVariance_;
    horizontalVarianceXY = 0.0;
  }

  clean();
  staticMap_.setTimestamp(timestamp.toNSec()); // Point cloud stores time in microseconds.

  const ros::WallDuration duration = ros::WallTime::now() - methodStartTime;
  ROS_INFO("Raw map has been updated with a new point cloud in %f s.", duration.toSec());
  return true;
}

bool DSElevationMap::ds_update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY,
                          const grid_map::Matrix& horizontalVarianceUpdateXY, const ros::Time& time)
{
    int testindex = 0;
//    for(SubmapIterator areaIterator(staticMap_, Index(0,0), staticMap_.getSize()); !areaIterator.isPastEnd(); ++areaIterator) {
//        if(staticMap_.isValid(*areaIterator)){
//            ++testindex;
//        }
//    }
//    cout<<"update predict static map valid num:"<<testindex<<endl;
    const auto& size = staticMap_.getSize();

    if (!((Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all()
        && (Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all()
        && (Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all()
        && (Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
      ROS_ERROR("The size of the update matrices does not match.");
      return false;
    }

    staticMap_.get("variance") += varianceUpdate;
    staticMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
    staticMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
    staticMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
    dynamicMap_.get("variance") += varianceUpdate;
    dynamicMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
    dynamicMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
    dynamicMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
    clean();
    staticMap_.setTimestamp(time.toNSec());
    dynamicMap_.setTimestamp(time.toNSec());

    //problem in this function
//    testindex = 0;
//    for(SubmapIterator areaIterator(staticMap_, Index(0,0), staticMap_.getSize()); !areaIterator.isPastEnd(); ++areaIterator) {
//        if(staticMap_.isValid(*areaIterator)){
//            ++testindex;
//        }
//    }
//    cout<<"after update predict static map valid num:"<<testindex<<endl;
    return true;
}

bool DSElevationMap::update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY,
                          const grid_map::Matrix& horizontalVarianceUpdateXY, const ros::Time& time)
{
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = staticMap_.getSize();

  if (!((Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all()
      && (Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all()
      && (Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all()
      && (Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
    ROS_ERROR("The size of the update matrices does not match.");
    return false;
  }

  staticMap_.get("variance") += varianceUpdate;
  staticMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  staticMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  staticMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
  clean();
  staticMap_.setTimestamp(time.toNSec());

  return true;
}

bool DSElevationMap::fusePreToStaDyn()
{
    ROS_INFO("fuse to static dynamic map");
    int64_t begin = clock(), end;
    finalMap_.clearBasic();
    int s_update_null = 0, s_update = 0, d_update = 0, dts_update = 0;
    for(SubmapIterator areaIterator(preMap_, Index(0,0), preMap_.getSize()); !areaIterator.isPastEnd(); ++areaIterator) {
        if(!preMap_.isValid(*areaIterator)){
            // clear dynamic layer
            dynamicMap_.at("error", *areaIterator) = 0;
            dynamicMap_.at("variance", *areaIterator) = -1;
            // update final layer
            if(staticMap_.isValid(*areaIterator) && staticMap_.at("variance", *areaIterator) > 0){
                finalMap_.at("variance", *areaIterator) = staticMap_.at("variance", *areaIterator);
                finalMap_.at("elevation", *areaIterator) = staticMap_.at("elevation", *areaIterator);
            }
            continue;
        }
        Index index = areaIterator.getSubmapIndex();  //the same as *areaIterator
        //cout<<"update index:"<<index<<"*inter:"<<*areaIterator<<endl;
        auto& s_elevation = staticMap_.at("elevation", index);
        auto& s_variance = staticMap_.at("variance", index);
        auto& s_lowestScanPoint = staticMap_.at("lowest_scan_point", index);
        auto& s_sensorXatLowestScan = staticMap_.at("sensor_x_at_lowest_scan", index);
        auto& s_sensorYatLowestScan = staticMap_.at("sensor_y_at_lowest_scan", index);
        auto& s_sensorZatLowestScan = staticMap_.at("sensor_z_at_lowest_scan", index);

        auto& d_error = dynamicMap_.at("error", index);
        auto& d_variance = dynamicMap_.at("variance", index);
        auto& d_lowestScanPoint = dynamicMap_.at("lowest_scan_point", index);
        auto& d_sensorXatLowestScan = dynamicMap_.at("sensor_x_at_lowest_scan", index);
        auto& d_sensorYatLowestScan = dynamicMap_.at("sensor_y_at_lowest_scan", index);
        auto& d_sensorZatLowestScan = dynamicMap_.at("sensor_z_at_lowest_scan", index);

        auto& p_elevation = preMap_.at("elevation", index);
        auto& p_variance = preMap_.at("variance", index);
        auto& p_lowestScanPoint = preMap_.at("lowest_scan_point", index);
        auto& p_sensorXatLowestScan = preMap_.at("sensor_x_at_lowest_scan", index);
        auto& p_sensorYatLowestScan = preMap_.at("sensor_y_at_lowest_scan", index);
        auto& p_sensorZatLowestScan = preMap_.at("sensor_z_at_lowest_scan", index);
        auto& p_error = preMap_.at("error_static", *areaIterator);
        auto& p_to_dyn_or_sta = preMap_.at("to_dyn_or_sta", *areaIterator);

        if(p_to_dyn_or_sta < 0.5){
            // to static
            ++s_update;
            if(staticMap_.isValid(*areaIterator) && s_variance >= 0){
                // Fuse measurement with elevation map data.
                s_elevation = (s_variance * p_elevation + p_variance * s_elevation) / (s_variance + p_variance);
                s_variance = (s_variance * p_variance) / (s_variance + p_variance);
                if(p_lowestScanPoint < s_lowestScanPoint){
                    s_lowestScanPoint = p_lowestScanPoint;
                    s_sensorXatLowestScan = p_sensorXatLowestScan;
                    s_sensorYatLowestScan = p_sensorYatLowestScan;
                    s_sensorZatLowestScan = p_sensorZatLowestScan;
                }
//                cout<<"update to sta(svar, sele, pele, pvar): "<<s_variance
//                   <<", "<<s_elevation<<", "<<p_elevation<<", "<<p_variance<<endl;
            }
            else{
                //put observation in directly
                ++s_update_null;
                s_elevation = p_elevation;
                s_variance = p_variance;
                s_lowestScanPoint = p_lowestScanPoint;
                s_sensorXatLowestScan = p_sensorXatLowestScan;
                s_sensorYatLowestScan = p_sensorYatLowestScan;
                s_sensorZatLowestScan = p_sensorZatLowestScan;
            }
            d_error = 0;
            d_variance = -1;
            finalMap_.at("variance", *areaIterator) = s_variance;
            finalMap_.at("elevation", *areaIterator) = s_elevation;
        }
        else{
            // to dynamic
            ++d_update;
            if(dynamicMap_.isValid(*areaIterator) && d_variance >= 0){
                double err_dyn_pre = p_error - d_error;
                if(fabs(err_dyn_pre) > error_measure_threshold){
                    // maybe something moving
                    // todo change to other model
                    d_error = p_error;
                    d_variance = p_variance;
                    d_lowestScanPoint = p_lowestScanPoint;
                    d_sensorXatLowestScan = p_sensorXatLowestScan;
                    d_sensorYatLowestScan = p_sensorYatLowestScan;
                    d_sensorZatLowestScan = p_sensorZatLowestScan;
                }
                else{
                    d_error = (d_variance * p_error + p_variance * d_error) / (d_variance + p_variance);
                    d_variance = (d_variance * p_variance) / (d_variance + p_variance);
                    if(p_lowestScanPoint < d_lowestScanPoint){
                        d_lowestScanPoint = p_lowestScanPoint;
                        d_sensorXatLowestScan = p_sensorXatLowestScan;
                        d_sensorYatLowestScan = p_sensorYatLowestScan;
                        d_sensorZatLowestScan = p_sensorZatLowestScan;
                    }
//                    cout<<"update to dyn(dvar, derr, perr, pvar): "<<d_variance
//                       <<", "<<d_error<<", "<<p_error<<", "<<p_variance<<endl;
                }

            }
            else{
                //put observation in directly
                d_error = p_error;
                d_variance = p_variance;
                d_lowestScanPoint = p_lowestScanPoint;
                d_sensorXatLowestScan = p_sensorXatLowestScan;
                d_sensorYatLowestScan = p_sensorYatLowestScan;
                d_sensorZatLowestScan = p_sensorZatLowestScan;
            }
            //todo: dynamic update to static
            if(d_variance > 0 && d_variance < dtos_threshold){
                s_elevation += d_error;
                s_variance = d_variance;
                d_error = 0;
                d_variance = -1;
                ++dts_update;
            }
            // add static and dynamic to final
            finalMap_.at("variance", *areaIterator) = d_variance;
            finalMap_.at("elevation", *areaIterator) = s_elevation + d_error;
        }
    }
    end = clock();
    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    cout<<"fuse to dynamic static map time: "<<search_time<<"s."<<endl;
//    cout<<"update to static total: "<<s_update<<", to static null: "
//       <<s_update_null<<", to dynamic: "<<d_update<<", dyn to sta: "
//      <<dts_update<<endl;
}

/*
bool DSElevationMap::fuseAll()
{
  ROS_DEBUG("Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(Index(0, 0), fusedMap_.getSize());
}

bool DSElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
  ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)",
            position[0], position[1], length[0], length[1]);

  Index topLeftIndex;
  Index submapBufferSize;

  // These parameters are not used in this function.
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                       requestedIndexInSubmap, position, length, staticMap_.getLength(),
                       staticMap_.getPosition(), staticMap_.getResolution(), staticMap_.getSize(),
                       staticMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize);
}

bool DSElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size)
{
    ROS_INFO("start fuse map.");
  ROS_DEBUG("Fusing elevation map...");

  // Nothing to do.
  if ((size == 0).any()) return false;

  // Initializations.
  const ros::WallTime methodStartTime(ros::WallTime::now());
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = staticMap_;
  scopedLockForRawData.unlock();

  // More initializations.
  const double halfResolution = fusedMap_.getResolution() / 2.0;
  const float minimalWeight = std::numeric_limits<float>::epsilon() * (float) 2.0;
  // Conservative cell inclusion for ellipse iterator.
  const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) resetFusedData();

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) fusedMap_.move(rawMapCopy.getPosition());

  // For each cell in requested area.
  for (SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {

    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) continue;

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      // TODO.
      continue;
    }

    // Get size of error ellipse.
    const float& sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
    const float& sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
    const float& sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
    // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
    // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
    const double uncertaintyFactor = 2.486; // sqrt(6.18)
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

    Eigen::Array2d::Index maxEigenvalueIndex;
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex;
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    const Length ellipseLength =  2.0 * uncertaintyFactor * Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() + ellipseExtension;
    const double ellipseRotation(atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1), solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

    // Requested length and position (center) of submap in map.
    Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
    EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength, ellipseRotation);

    // Prepare data fusion.
    Eigen::ArrayXf means, weights;
    const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
    means.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);
    WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
    WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
    std::string maxEigenvalueLayer, minEigenvalueLayer;
    if (maxEigenvalueIndex == 0) {
      maxEigenvalueLayer = "horizontal_variance_x";
      minEigenvalueLayer = "horizontal_variance_y";
    } else {
      maxEigenvalueLayer = "horizontal_variance_y";
      minEigenvalueLayer = "horizontal_variance_x";
    }

    // For each cell in error ellipse.
    size_t i = 0;
    for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
      if (!rawMapCopy.isValid(*ellipseIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *ellipseIterator);

      // Compute weight from probability.
      Position absolutePosition;
      rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
      Eigen::Vector2d distanceToCenter = (rotationMatrix * (absolutePosition - requestedSubmapPosition)).cwiseAbs();

      float probability1 =
            cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0, maxStandardDeviation)
          - cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0, maxStandardDeviation);
      float probability2 =
            cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0, minStandardDeviation)
          - cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0, minStandardDeviation);

      const float weight = max(minimalWeight, probability1 * probability2);
      weights[i] = weight;
      const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
      lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
      upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

      i++;
    }

    if (i == 0) {
      // Nothing to fuse.
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("lower_bound", *areaIterator) = rawMapCopy.at("elevation", *areaIterator) - 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("upper_bound", *areaIterator) = rawMapCopy.at("elevation", *areaIterator) + 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      continue;
    }

    // Fuse.
    means.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();

    if (!std::isfinite(mean)) {
      ROS_ERROR("Something went wrong when fusing the map: Mean = %f", mean);
      continue;
    }

    // Add to fused map.
    fusedMap_.at("elevation", *areaIterator) = mean;
    lowerBoundDistribution.compute();
    upperBoundDistribution.compute();
    fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01); // TODO
    fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99); // TODO
    // TODO Add fusion of colors.
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  const ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
  ROS_INFO("Elevation map has been fused in %f s.", duration.toSec());

  return true;
}
*/

bool DSElevationMap::clear()
{
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  staticMap_.clearAll();
  staticMap_.resetTimestamp();
  dynamicMap_.clearAll();
  dynamicMap_.resetTimestamp();
  preMap_.clearAll();
  preMap_.resetTimestamp();
  finalMap_.clearAll();
  finalMap_.resetTimestamp();
  return true;
}

void DSElevationMap::visibilityCleanup(const ros::Time& updatedTime)
{
}

void DSElevationMap::move(const Eigen::Vector2d& position)
{
  //boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  std::vector<BufferRegion> newRegions;
  //ROS_INFO("static map has been moved to position (%f, %f).", staticMap_.getPosition().x(), staticMap_.getPosition().y());

  if (staticMap_.move(position, newRegions)) {
    preMap_.move(position);
    dynamicMap_.move(position);
    finalMap_.move(position);
    ROS_INFO("Elevation map has been moved to position (%f, %f).", staticMap_.getPosition().x(), staticMap_.getPosition().y());
  }
  //ROS_INFO("preMap_ map has been moved to position (%f, %f).", preMap_.getPosition().x(), preMap_.getPosition().y());
}

void DSElevationMap::publishAllElevationMap()
{
    publishDynamicElevationMap();
    publishFinalElevationMap();
    publishPresentElevationMap();
    publishStaticElevationMap();
}

bool DSElevationMap::publishDynamicElevationMap()
{
    grid_map::GridMap rawMapCopy = dynamicMap_;
    rawMapCopy.erase("lowest_scan_point");
    rawMapCopy.erase("sensor_x_at_lowest_scan");
    rawMapCopy.erase("sensor_y_at_lowest_scan");
    rawMapCopy.erase("sensor_z_at_lowest_scan");
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(rawMapCopy, message);
    elevationMapDynamicPublisher_.publish(message);
    return true;
}

bool DSElevationMap::publishStaticElevationMap()
{
    grid_map::GridMap rawMapCopy = staticMap_;
    rawMapCopy.erase("lowest_scan_point");
    rawMapCopy.erase("sensor_x_at_lowest_scan");
    rawMapCopy.erase("sensor_y_at_lowest_scan");
    rawMapCopy.erase("sensor_z_at_lowest_scan");
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(rawMapCopy, message);
    elevationMapStaticPublisher_.publish(message);
    return true;
}

bool DSElevationMap::publishPresentElevationMap()
{
    grid_map::GridMap rawMapCopy = preMap_;
    rawMapCopy.erase("lowest_scan_point");
    rawMapCopy.erase("sensor_x_at_lowest_scan");
    rawMapCopy.erase("sensor_y_at_lowest_scan");
    rawMapCopy.erase("sensor_z_at_lowest_scan");
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(rawMapCopy, message);
    elevationMapPresentPublisher_.publish(message);
    return true;
}

bool DSElevationMap::publishFinalElevationMap()
{
    grid_map::GridMap MapCopy = finalMap_;
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(MapCopy, message);
    elevationMapFinalPublisher_.publish(message);
    return true;
}

//bool DSElevationMap::publishRawElevationMap()
//{
//  if (!hasRawMapSubscribers()) return false;
//  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
//  grid_map::GridMap rawMapCopy = staticMap_;
//  scopedLock.unlock();
//  rawMapCopy.erase("lowest_scan_point");
//  rawMapCopy.erase("sensor_x_at_lowest_scan");
//  rawMapCopy.erase("sensor_y_at_lowest_scan");
//  rawMapCopy.erase("sensor_z_at_lowest_scan");
//  rawMapCopy.add("standard_deviation", rawMapCopy.get("variance").array().sqrt().matrix());
//  rawMapCopy.add("horizontal_standard_deviation", (rawMapCopy.get("horizontal_variance_x") + rawMapCopy.get("horizontal_variance_y")).array().sqrt().matrix());
//  rawMapCopy.add("two_sigma_bound", rawMapCopy.get("elevation") + 2.0 * rawMapCopy.get("variance").array().sqrt().matrix());
//  grid_map_msgs::GridMap message;
//  GridMapRosConverter::toMessage(rawMapCopy, message);
//  elevationMapRawPublisher_.publish(message);
//  ROS_DEBUG("Elevation map raw has been published.");
//  return true;
//}

//bool DSElevationMap::publishFusedElevationMap()
//{
//  if (!hasFusedMapSubscribers()) return false;
//  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
//  GridMap fusedMapCopy = fusedMap_;
//  scopedLock.unlock();
//  fusedMapCopy.add("uncertainty_range", fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
//  grid_map_msgs::GridMap message;
//  GridMapRosConverter::toMessage(fusedMapCopy, message);
//  elevationMapFusedPublisher_.publish(message);
//  ROS_DEBUG("Elevation map (fused) has been published.");
//  return true;
//}

bool DSElevationMap::publishVisibilityCleanupMap()
{
  if (visbilityCleanupMapPublisher_.getNumSubscribers() < 1) return false;
  boost::recursive_mutex::scoped_lock scopedLock(visibilityCleanupMapMutex_);
  grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
  scopedLock.unlock();
  visibilityCleanupMapCopy.erase("elevation");
  visibilityCleanupMapCopy.erase("variance");
  visibilityCleanupMapCopy.erase("horizontal_variance_x");
  visibilityCleanupMapCopy.erase("horizontal_variance_y");
  visibilityCleanupMapCopy.erase("horizontal_variance_xy");
  visibilityCleanupMapCopy.erase("color");
  visibilityCleanupMapCopy.erase("time");
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(visibilityCleanupMapCopy, message);
  visbilityCleanupMapPublisher_.publish(message);
  ROS_DEBUG("Visibility cleanup map has been published.");
  return true;
}

grid_map::GridMap& DSElevationMap::getStaticGridMap()
{
  return staticMap_;
}

grid_map::GridMap& DSElevationMap::getDynamicGridMap()
{
  return dynamicMap_;
}

grid_map::GridMap& DSElevationMap::getPresentGridMap()
{
  return preMap_;
}

grid_map::GridMap& DSElevationMap::getFinalGridMap()
{
  return finalMap_;
}

ros::Time DSElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(staticMap_.getTimestamp());
}

ros::Time DSElevationMap::getTimeOfLastFusion()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return ros::Time().fromNSec(staticMap_.getTimestamp());
}

const kindr::HomTransformQuatD& DSElevationMap::getPose()
{
  return pose_;
}

bool DSElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position)
{
  kindr::Position3D positionInGridFrame;
  if (!staticMap_.getPosition3("elevation", index, positionInGridFrame.vector())) return false;
  position = pose_.transform(positionInGridFrame);
  return true;
}

boost::recursive_mutex& DSElevationMap::getFusedDataMutex()
{
  return fusedMapMutex_;
}

boost::recursive_mutex& DSElevationMap::getRawDataMutex()
{
  return rawMapMutex_;
}

bool DSElevationMap::clean()
{
//  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  staticMap_.get("variance") = staticMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));
  staticMap_.get("horizontal_variance_x") = staticMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  staticMap_.get("horizontal_variance_y") = staticMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));

  dynamicMap_.get("variance") = dynamicMap_.get("variance").unaryExpr(VarianceClampOperator<float>(minVariance_, maxVariance_));
  dynamicMap_.get("horizontal_variance_x") = dynamicMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  dynamicMap_.get("horizontal_variance_y") = dynamicMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void DSElevationMap::resetFusedData()
{
//  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  finalMap_.clearAll();
  finalMap_.resetTimestamp();
}

void DSElevationMap::setFrameId(const std::string& frameId)
{
  staticMap_.setFrameId(frameId);
  dynamicMap_.setFrameId(frameId);
  preMap_.setFrameId(frameId);
  finalMap_.setFrameId(frameId);
}

const std::string& DSElevationMap::getFrameId()
{
  return staticMap_.getFrameId();
}

/*
bool DSElevationMap::hasRawMapSubscribers() const
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

bool DSElevationMap::hasFusedMapSubscribers() const
{
  if (elevationMapFusedPublisher_.getNumSubscribers() < 1) return false;
  return true;
}
*/
void DSElevationMap::underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap)
{

}

float DSElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

} /* namespace */
