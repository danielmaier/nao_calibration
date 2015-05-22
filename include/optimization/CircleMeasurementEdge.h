/*
 * CircleMeasurementEdge.h
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#ifndef CIRCLEMEASUREMENTEDGE_H_
#define CIRCLEMEASUREMENTEDGE_H_

#include <kinematic_calibration/measurementData.h>
#include <Eigen/Dense>
#include <vector>
#include "../data_capturing/CircleDetection.h"

#include "MeasurementEdge.h"

#define SQRT_2_PI 2.506628275

namespace kinematic_calibration {

typedef Matrix<double, 5, 1> Vector5d;

/**
 * Represents an edge for detected circles within an image.
 */
class CircleMeasurementEdge: public MeasurementEdge<2, CircleMeasurementEdge> {
public:
	CircleMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain, double radius);
	virtual ~CircleMeasurementEdge();

	/**
	 * Calculates the error vector.
	 * _error[0] is the distance between measured and estimated center of the circle.
	 * _error[1-4] are the distances between the estimated points on the ellipse
	 * and the circle contour, using the measured circle radius.
	 * @param cameraToMarker
	 */
	void setError(tf::Transform cameraToMarker);

protected:
	/**
	 * Takes the measurement image and tries to extract the circle's contours.
	 * The result will be a binary image having pixels being part of a contour
	 * set to the color white and other pixels set to black.
	 */
	void calculateMeasurementConturs();

	/**
	 * Calculates the estimated ellipse.
	 * @param cameraToMarker Transformation to use.
	 */
	void calculateEstimatedEllipse(const tf::Transform& cameraToMarker);

	/**
	 * The measured center of the circle/ellipse.
	 */
	Vector2d measurementCenter;

	/**
	 * The estimated ellipse.
	 * [x0, y0, a, b, -alpha]
	 */
	Vector5d estimatedEllipse;

	/**
	 * The estimated contour of the circle.
	 */
	vector<Vector2d> estimatedContour;

	/**
	 * The measured radius of the circle [m].
	 */
	double radius;

	/**
	 * A binary image having pixels being part of a contour
	 * set to the color white and other pixels set to black.
	 */
	//cv::Mat contourImage;
	CircleDetection circleDetection;

private:
	Vector5d projectSphereToImage(const Eigen::Vector3d& pos,
			const Eigen::Matrix<double, 3, 4>& projectionMatrix,
			const double& sphereRadius) const;

	Vector2d getEllipsePoint(const double& x0, const double& y0,
			const double& a, const double& b, const double& alpha,
			const double& t) const;

	void getEllipseData(double& x0, double& y0, double& a, double& b,
			double& alpha);
};

class LikelihoodCircleMeasurementEdge: public MeasurementEdge<1,
		LikelihoodCircleMeasurementEdge> {
public:
	/**
	 * Constructor.
	 */
	LikelihoodCircleMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain, double radius);

	/**
	 * Destructor.
	 */
	virtual ~LikelihoodCircleMeasurementEdge();

	/**
	 * Sets the error, based on the likelihood.
	 * @param cameraToMarker Transformation form camera to marker, based on the current state.
	 */
	void setError(tf::Transform cameraToMarker);

private:
	bool compute_likelihood(const cv::Mat & edge_img, const cv::Mat & ori,
			tf::Transform & pose, cv::Mat & outputImg, double & likelihood,
			double & residual, bool draw = false);

	double matching_likelihood(double res, double max_range);

        // calculates the canny and orientation image for the circle likelihood function
	void calculateCannyAndOrientationImg();

	/**
	 * Radius of the circle.
	 */
	double radius;

	/**
	 * Original measurement image.
	 */
	cv::Mat measurementImg;

	/**
	 * Measurement image after applying the canny filter.
	 */
	cv::Mat cannyImg;

	/**
	 * Orientation image.
	 */
	cv::Mat orientationImg;

	/**
	 * Output image.
	 */
	cv::Mat outputImage;

	// TODO: info/doxygen
	double m_search_length_1d;
	double m_angle_thresh;
	double m_ellipse_sampling_increment;
	std::vector<cv::Vec3b> m_colors;
	bool m_use_beam_model_likelihood;
	double m_zHit;
	double m_zRand;
	double m_zMax;
	double m_sigmaHit;
	double m_lambda_r;

};

void sphere_to_quadric(const Eigen::Vector3d & pos,
		const Eigen::Matrix<double, 3, 4> & projection_matrix,
		double sphere_radius, Eigen::Matrix3d & Q);

void quadric_to_sphere(const Eigen::Matrix3d & Q,
		const Eigen::Matrix<double, 3, 4> & projection_matrix,
		double sphere_radius, Eigen::Vector3d & a);

void tf_to_opencv(const tf::Transform &pose, cv::Mat_<double> & R3,
		cv::Mat_<double> & T3);

bool ellipse_from_quadric(const Eigen::Matrix3d & CstarI, Vector5d & z,
		Eigen::Matrix2d & U);

vector<cv::Vec3b> generate_heatmap(bool wrap);

cv::Vec3b apply_heatmap(const std::vector<cv::Vec3b> & colors, int max_val,
		double val);

bool test_angle(const cv::Mat & ori, float angle_thresh, cv::Rect & rect,
		cv::LineIterator & it, cv::Mat & outputImage, double line_orientation);

void orientation_image(const cv::Mat & cimg,
		const std::vector<cv::Vec3b> & colors, cv::Mat & outputImg);

cv::Mat orientationMap(const cv::Mat& mag, const cv::Mat& ori, double thresh,
		const std::vector<cv::Vec3b> & colors);

class Correspondence {
public:
	Correspondence() {
	}
	;
	std::pair<cv::Point3d, cv::Point2d> point3d_to_point2d;
	std::pair<cv::Point2d, cv::Point2d> point2d_to_point2d;
	double residual;
	cv::Point2d normal;

};

} /* namespace kinematic_calibration */

#endif /* CIRCLEMEASUREMENTEDGE_H_ */
