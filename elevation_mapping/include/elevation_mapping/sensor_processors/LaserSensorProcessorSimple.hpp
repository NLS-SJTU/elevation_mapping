/*
 * LaserSensorProcessor.hpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

namespace elevation_mapping {

/*!
 * Sensor processor for laser range sensors.
 */
class LaserSensorProcessorSimple : public SensorProcessorBase
{

public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  LaserSensorProcessorSimple(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
  virtual ~LaserSensorProcessorSimple();

  /*!
   * just to filter here
   */
  virtual bool preprocess(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudInput,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapFrame);

  /*!
   * compute necessary variables before computing variance
   */
  virtual void precompute(const Eigen::Matrix<double, 6, 6>& robotPoseCovariance);

  /*!
   * compute variance of one point
   */
  virtual double computeVarOfOnePoint(pcl::PointXYZRGB point);

private:

  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  virtual bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      Eigen::VectorXf& variances);

  /*!
   * some necessary variables
   */
  Eigen::RowVector3f projectionVector;
  Eigen::RowVector3f sensorJacobian;
  Eigen::Matrix3f rotationVariance;
  Eigen::Matrix3f C_BM_transpose;
  Eigen::RowVector3f P_mul_C_BM_transpose;
  Eigen::Matrix3f C_SB_transpose;
  Eigen::Matrix3f B_r_BS_skew;

};


} /* namespace elevation_mapping */
