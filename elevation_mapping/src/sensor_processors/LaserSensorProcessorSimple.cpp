/*
 * KinectSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/LaserSensorProcessorSimple.hpp>

#include <pcl/filters/passthrough.h>
#include <vector>
#include <limits>
#include <string>

namespace elevation_mapping {

/*!
 * Anisotropic laser range sensor model:
 * standardDeviationInBeamDirection = minRadius
 * standardDeviationOfBeamRadius = beamConstant + beamAngle * measurementDistance
 *
 * Taken from: Pomerleau, F.; Breitenmoser, A; Ming Liu; Colas, F.; Siegwart, R.,
 * "Noise characterization of depth sensors for surface inspections,"
 * International Conference on Applied Robotics for the Power Industry (CARPI), 2012.
 */

LaserSensorProcessorSimple::LaserSensorProcessorSimple(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{
    readParameters();
}

LaserSensorProcessorSimple::~LaserSensorProcessorSimple()
{

}

bool LaserSensorProcessorSimple::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/min_radius", sensorParameters_["min_radius"], 0.0);
  nodeHandle_.param("sensor_processor/beam_angle", sensorParameters_["beam_angle"], 0.0);
  nodeHandle_.param("sensor_processor/beam_constant", sensorParameters_["beam_constant"], 0.0);
  return true;
}

bool LaserSensorProcessorSimple::computeVariances(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
{
	variances.resize(pointCloud->size());

	// Projection vector (P).
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (size_t i = 0; i < pointCloud->size(); ++i) {
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
		float measurementDistance = pointVector.norm();

		// Compute sensor covariance matrix (Sigma_S) with sensor model.
		float varianceNormal = pow(sensorParameters_.at("min_radius"), 2);
		float varianceLateral = pow(sensorParameters_.at("beam_constant") + sensorParameters_.at("beam_angle") * measurementDistance, 2);
		Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
		sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

		// Robot rotation Jacobian (J_q).
		const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
		Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

		// Measurement variance for map (error propagation law).
		heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
		heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

		// Copy to list.
		variances(i) = heightVariance;
	}

	return true;
}

// original code thinks point cloud is not in the same frame as sensor frame(is not wired?)
// thus, here we skip some transformation steps.
bool LaserSensorProcessorSimple::preprocess(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudInput,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapFrame){
    ros::Time timeStamp;
    timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
    if (!updateTransformations(timeStamp)) return false;
    filterPointCloud(pointCloudInput);

    if (!transformPointCloud(pointCloudInput, pointCloudMapFrame, mapFrameId_)) return false;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds({pointCloudMapFrame, pointCloudInput});
    removePointsOutsideLimits(pointCloudMapFrame, pointClouds);

    return true;
}


void LaserSensorProcessorSimple::precompute(const Eigen::Matrix<double, 6, 6>& robotPoseCovariance){
    // Projection vector (P).
    projectionVector = Eigen::RowVector3f::UnitZ();

    // Sensor Jacobian (J_s).
    sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();
    // std::cout<<"sensor jaco: "<<sensorJacobian<<std::endl;

    // Robot rotation covariance matrix (Sigma_q).
    rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
    C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
    P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
    C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
    B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));
}

// points in sensor frame
double LaserSensorProcessorSimple::computeVarOfOnePoint(pcl::PointXYZRGB point){
    // Preparation.
    Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
    double heightVariance = 0.0; // sigma_p

    // Measurement distance.
    float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    float varianceNormal = pow(sensorParameters_.at("min_radius"), 2);
    float varianceLateral = pow(sensorParameters_.at("beam_constant") + sensorParameters_.at("beam_angle") * measurementDistance, 2);
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    return heightVariance;
}

} /* namespace */
