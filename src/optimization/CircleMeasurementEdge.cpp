/*
 * CircleMeasurementEdge.cpp
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#include "../../include/optimization/CircleMeasurementEdge.h"

#include <boost/format/format_class.hpp>
#include <boost/format/format_fwd.hpp>
#include <kinematic_calibration/measurementData.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "../../include/data_capturing/CircleDetection.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace Eigen;
using namespace std;
using namespace cv;

namespace kinematic_calibration {

CircleMeasurementEdge::CircleMeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain, double radius) :
		MeasurementEdge<2, CircleMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain), radius(radius) {
	calculateMeasurementConturs();
}

CircleMeasurementEdge::~CircleMeasurementEdge() {
	// nothing to do
}

void CircleMeasurementEdge::setError(tf::Transform cameraToMarker) {
	this->calculateEstimatedEllipse(cameraToMarker);

	double xe, ye, xm, ym, r, xec, yec;
	r = this->measurement.marker_data[CircleDetection::idx_r];

	// distance of center
	xec = this->estimatedEllipse[0];
	yec = this->estimatedEllipse[1];
	xm = this->measurement.marker_data[CircleDetection::idx_x];
	ym = this->measurement.marker_data[CircleDetection::idx_y];

	this->_error[0] = sqrt((xec - xm) * (xec - xm) + ((yec - ym) * (yec - ym)));

	// contour
	double x0, y0, a, b, alpha;
	this->getEllipseData(x0, y0, a, b, alpha);
	this->_error[1] = 0;
	int numOfPoints = 16;
	int matchingPoints = 0;
	cv::Mat tempImg(this->circleDetection.getGaussImg());
	cv::Mat colorImg;
	cv::cvtColor(tempImg, colorImg, CV_GRAY2BGR);
	for (int i = 0; i < numOfPoints; i++) {
		// check whether the current point is part
		// of the contour pixels in the measurement image
		Vector2d point = this->getEllipsePoint(x0, y0, a, b, -alpha,
				i * 2 * M_PI / numOfPoints);
		int x = (int) point[0];
		int y = (int) point[1];
		if (this->circleDetection.getGaussImg().ptr<uchar>(y)[1 * x] > 0) {
			// pixel is white i.e. part of a contour
			matchingPoints++;
			colorImg.ptr<uchar>(y)[3 * x] = 255;
		} else {
			colorImg.ptr<uchar>(y)[3 * x + 1] = 255;
			colorImg.ptr<uchar>(y)[3 * x + 2] = 255;
		}
	}
	this->_error[1] = numOfPoints - matchingPoints;

	// TODO: remove following debug code
	//cout << "Matching points: " << matchingPoints << endl;
	stringstream ss1, ss2;
	ss1 << "/tmp/" << measurement.id << ".contours.jpg";
	cv::imwrite(ss1.str(), colorImg);

	Vector5d ep = this->estimatedEllipse;
	cv_bridge::CvImageConstPtr cv_ptr;
	sensor_msgs::ImageConstPtr imgMsgPtr(
			new sensor_msgs::Image(this->measurement.image));
	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(imgMsgPtr,
				sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv::Mat image = circleDetection.getGaussImg();	// cv_ptr->image;
	cv::ellipse(image, cv::Point2d(ep(0), ep(1)), cv::Size(ep(2), ep(3)),
			ep(4) * 180.0 / M_PI, 0, 360, cv::Scalar(255, 0, 0), 2, 8);
	ss2 << "/tmp/" << measurement.id << ".ellipse.jpg";
	cv::imwrite(ss2.str(), image);

}

void CircleMeasurementEdge::calculateMeasurementConturs() {
	vector<cv::Vec3f> vec;
	cv::Mat out_image;
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	this->circleDetection.msgToImg(
			boost::shared_ptr<sensor_msgs::Image>(
					new sensor_msgs::Image(this->measurement.image)),
			out_image);
	this->circleDetection.detect(out_image, vec);
}

void CircleMeasurementEdge::calculateEstimatedEllipse(
		const tf::Transform& cameraToMarker) {
	// get current estimated camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);
	sensor_msgs::CameraInfo cameraInfo = cameraIntrinsicsVertex->estimate();
	Eigen::Matrix<double, 3, 4> projectionMatrix;
	projectionMatrix << cameraInfo.P[0], cameraInfo.P[1], cameraInfo.P[2], cameraInfo.P[3], cameraInfo.P[4], cameraInfo.P[5], cameraInfo.P[6], cameraInfo.P[7], cameraInfo.P[8], cameraInfo.P[9], cameraInfo.P[10], cameraInfo.P[11];

	// pos = translation of the current estimated transformation
	Eigen::Vector3d pos;
	tf::Transform markerToCamera = cameraToMarker.inverse();
	pos << markerToCamera.getOrigin().getX(), markerToCamera.getOrigin().getY(), markerToCamera.getOrigin().getZ();

	// estimated ellipse data
	this->estimatedEllipse = this->projectSphereToImage(pos, projectionMatrix,
			radius);
}

void CircleMeasurementEdge::getEllipseData(double& x0, double& y0, double& a,
		double& b, double& alpha) {
	x0 = this->estimatedEllipse[0];
	y0 = this->estimatedEllipse[1];
	a = this->estimatedEllipse[2];
	b = this->estimatedEllipse[3];
	alpha = -this->estimatedEllipse[4];
}

Vector2d CircleMeasurementEdge::getEllipsePoint(const double& x0,
		const double& y0, const double& a, const double& b, const double& alpha,
		const double& t) const {
	return Eigen::Vector2d(
			x0 + a * cos(t) * cos(alpha) - b * sin(t) * sin(alpha),
			y0 + a * cos(t) * sin(alpha) + b * sin(t) * cos(alpha));
}

Vector5d CircleMeasurementEdge::projectSphereToImage(const Eigen::Vector3d& pos,
		const Eigen::Matrix<double, 3, 4>& projectionMatrix,
		const double& sphereRadius) const {
	Vector5d z_pred;
	double gamma = pos.transpose() * pos - sphereRadius * sphereRadius;
	double beta = 1.0 / (pos.transpose() * pos - gamma);
	Eigen::Matrix4d Sstar;
	Sstar << Eigen::Matrix3d::Identity() - beta * pos * pos.transpose(), -beta
			* pos, -beta * pos.transpose(), -beta;
	//cout << "Sstar:\n" << Sstar << endl;
	Eigen::Matrix3d Cstar = projectionMatrix * Sstar
			* projectionMatrix.transpose();
	Eigen::Matrix3d CstarI = Cstar.inverse();
	//cout << "CstarI:\n" << CstarI << endl;
	double av = CstarI(0, 0);
	double bv = CstarI(0, 1);
	double cv = CstarI(0, 2);
	double dv = CstarI(1, 1);
	double ev = CstarI(1, 2);
	double fv = CstarI(2, 2);
	Eigen::Matrix2d A;
	A << av, bv, bv, dv;
	Eigen::Vector2d B;
	B << 2 * cv, 2 * ev;
	if (av * dv - bv * bv <= 0) {
		cerr
				<< boost::format(
						"This is not an ellipse at (%6.3f, %6.3f, %6.3f")
						% pos(0) % pos(1) % pos(2) << endl;
		throw std::runtime_error("This is not an ellipse");
	}

	Eigen::Vector2d K2;
	double b1 = B(0);
	double b2 = B(1);
	double a11 = av;
	double a12 = bv;
	double a22 = dv;
	double denom = 2 * (a12 * a12 - a11 * a22);
	K2 << (a22 * b1 - a12 * b2) / denom, (a11 * b2 - a12 * b1) / denom;
	double mu = 1
			/ (a11 * K2(0) * K2(0) + 2 * a12 * K2(0) * K2(1)
					+ a22 * K2(1) * K2(1) - fv);
	double m11 = mu * a11;
	double m12 = mu * a12;
	double m22 = mu * a22;
	double la1 = ((m11 + m22) + sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0;
	double la2 = ((m11 + m22) - sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0;
	double b = 1 / sqrt(la1); // semi-minor axis
	double a = 1 / sqrt(la2); // semi-major axis
	Eigen::Vector2d U1, U2; //direction of minor and major axis
	if (m11 >= m22) {
		U1 << (la1 - m22), m12;
		U1 = U1 / U1.norm();
	} else {
		U1 << m12, (la1 - m11);
		U1 = U1 / U1.norm();
	}
	U2 << -U1(1), U1(0); //U2 is orthogonal to U1
	double theta2 = 0.5 * atan2(-2 * a12, (a22 - a11)); // angle between major axis and pos. x-axis
	Vector5d z2;
	z2 << K2(0), K2(1), a, b, theta2;

	return z2;
}

LikelihoodCircleMeasurementEdge::LikelihoodCircleMeasurementEdge(
		measurementData measurement, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain, double radius) :
		MeasurementEdge<1, LikelihoodCircleMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain), radius(radius), m_ellipse_sampling_increment(
				5.0), m_angle_thresh(25.0), m_search_length_1d(130.0 /*30.0*/), m_lambda_r(
				0.5), m_zHit(0.9), m_zRand(0.05), m_zMax(0.05), m_sigmaHit(1.0), m_use_beam_model_likelihood( false), m_colors(generate_heatmap(true)) {
	// Convert from ROS message to OpenCV image.
	CircleDetection circleDetection;
	circleDetection.msgToImg(
			boost::shared_ptr<sensor_msgs::Image>(
					new sensor_msgs::Image(this->measurement.image)),
			this->measurementImg);


        // Calculate canny and orientation image.
	calculateCannyAndOrientationImg();
}

LikelihoodCircleMeasurementEdge::~LikelihoodCircleMeasurementEdge() {
	// nothing to do
}

void LikelihoodCircleMeasurementEdge::setError(tf::Transform cameraToMarker) {
	bool draw = false;
	cvtColor(this->cannyImg, this->outputImage, CV_GRAY2BGR);

	//  calculate likelihood and set the error
	double residual, likelihood;
	tf::Transform markerToCamera = cameraToMarker.inverse();
	this->compute_likelihood(this->cannyImg, this->orientationImg,
			markerToCamera, this->outputImage, likelihood, residual, draw);
	//cout << "likelihood: " << likelihood << " residual: " << residual << endl;
	this->_error[0] = residual;
	//this->_error[0] = 1 / likelihood; // likelihood is always > 0.0

	if (draw) {
		cv::imshow("outputImage", outputImage);
		cv::waitKey();
	}
	if (debug) {
		string filename = "/tmp/" + this->measurement.id + "-error.jpg";
		cv::imwrite(filename, outputImage);
	}
}

void LikelihoodCircleMeasurementEdge::calculateCannyAndOrientationImg() {
	cv::Mat cimg = this->measurementImg;
	cv::Mat gray(cimg.rows, cimg.cols, CV_8UC1);
	cv::Mat gray_flt(cimg.rows, cimg.cols, CV_8UC1);
	cvtColor(cimg, gray, COLOR_BGR2GRAY);

	int canny_low = 40;
	int canny_high = 90;
	int canny_kernel = 1;
	int filter_knl = 5;
	int filter_type = 0;
	int canny_blur_size = 0;

	if (filter_type == 0)
		bilateralFilter(gray, gray_flt, filter_knl, filter_knl * 2,
				filter_knl / 2);
	else if (filter_type == 1)
		medianBlur(gray, gray_flt, filter_knl);
	else if (filter_type == 2)
		blur(gray, gray_flt, Size(filter_knl, filter_knl));
	else {
		gray_flt = gray;
	}
	if (canny_kernel < 1)
		canny_kernel = 1;
	cout << "canny_kernel is " << canny_kernel << " " << canny_kernel * 2 + 1
			<< endl;
	Canny(gray_flt, cannyImg, canny_low, canny_high, canny_kernel * 2 + 1); // 40, 100 seem good

	Mat Sx;
	Sobel(gray_flt, Sx, CV_32F, 1, 0, 5);

	Mat Sy;
	Sobel(gray_flt, Sy, CV_32F, 0, 1, 5);

	Mat mag;
	magnitude(Sx, Sy, mag);

        phase(Sx, Sy, this->orientationImg, true);
        


	//GaussianBlur( cannyImg, cannyImg, Size( 5, 5 ), 0, 0 ); // needed for problems with besenham raycasting (shooting normal through a (perpendicular) non-horizontal/non-vertical line)
	if (canny_blur_size > 0)
		blur(cannyImg, cannyImg,
				Size(canny_blur_size * 2 + 1, 1 + 2 * canny_blur_size));

	// make cannyImg a binary image (255/0)
	double mag_thresh = 1.0;
	int canny_thresh = 2;

	for (int i = 0; i < cannyImg.rows; ++i) {
		for (int j = 0; j < cannyImg.cols; ++j) {
			if (cannyImg.at<uchar>(i, j) > canny_thresh
					&& mag.at<float>(i, j) > mag_thresh)
				cannyImg.at<uchar>(i, j) = 255;
			else
				cannyImg.at<uchar>(i, j) = 0;
		}
	}

        this->outputImage = orientationMap( mag, this->orientationImg, mag_thresh, m_colors);
        this->outputImage.setTo(0, cannyImg==0);

	if (debug) {
		string filename = "/tmp/" + this->measurement.id + "-canny.jpg";
		cv::imwrite(filename, this->cannyImg);
		filename = "/tmp/" + this->measurement.id + "-ori.jpg";
		cv::imwrite(filename, this->outputImage);
	}
}

bool LikelihoodCircleMeasurementEdge::compute_likelihood(
		const cv::Mat& edge_img, const cv::Mat& ori, tf::Transform& pose,
		cv::Mat& outputImg, double& likelihood, double& residual, bool draw) {
	Eigen::Matrix3d Q;
	tf::Vector3 o = pose.getOrigin();
	Eigen::Vector3d pos;
	pos << o.x(), o.y(), o.z();

	// get current estimated camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);

	// project the sphere
	Eigen::Matrix<double, 3, 4> projection_matrix;
	sensor_msgs::CameraInfo cameraInfo = cameraIntrinsicsVertex->estimate();
	projection_matrix << cameraInfo.P[0], cameraInfo.P[1], cameraInfo.P[2], cameraInfo.P[3], cameraInfo.P[4], cameraInfo.P[5], cameraInfo.P[6], cameraInfo.P[7], cameraInfo.P[8], cameraInfo.P[9], cameraInfo.P[10], cameraInfo.P[11];

	//cout << pos << endl;
	sphere_to_quadric(pos, projection_matrix, radius, Q);
	// small test: compute pos from Q
	//cout << Q << endl;
	//cout << projection_matrix << endl;

	Eigen::Vector3d a2;
	quadric_to_sphere(Q, projection_matrix, radius, a2);
	Eigen::Matrix<double, 3, 2> acmp;
	acmp << pos, a2;
	// seems to work so far. now fit ellipse...
	// TODO: Fit ellipse from matches

	cv::Mat_<double> R3(3, 1);
	cv::Mat_<double> T3(3, 1);
	tf_to_opencv(pose, R3, T3); // call
	Rect rect(0, 0, cannyImg.cols, cannyImg.rows);

	Eigen::Matrix2d U;
	Vector5d ep;
	if (!ellipse_from_quadric(Q, ep, U)) {
		ROS_ERROR_STREAM(
				"compute_likelihood: Could not compute ellipse from this quadric\n" << Q);
		return false;
	}
	cv::Point2d U1(U(0, 0), U(0, 1)); //minor axis
	cv::Point2d U2(U(1, 0), U(1, 1)); //major axis
	cv::Point2d center(ep(0), ep(1));
	double thetar = ep(4);
	double theta = (ep(4) * 180.0 / M_PI); // theta < 0 means that the ellipse is rotated counter-clockwise in the image (because y points downwards)
	if (draw || debug)
		cv::ellipse(outputImage, center, cv::Size(ep(2), ep(3)), theta, 0, 360,
				cv::Scalar(255, 0, 0), 2, 8);

	// draw ellipse normals
	double ct = cos(thetar);
	double st = sin(thetar);
	double a = ep(2);
	double b = ep(3);

	double search_length_1d = m_search_length_1d;
	double angle_thresh = m_angle_thresh;
	int n_matched_points = 0;
	int n_sampled_points = 0;
	double rbar2 = 0.0;
	double log_likelihood = 0;
	vector<Correspondence> correspondences;
	if (draw)
		cout << "theta is : " << theta << endl;

	for (double t = 0; t < 2 * M_PI;
			t += m_ellipse_sampling_increment * M_PI / 180.0) {
		++n_sampled_points;
		double x = a * ct * cos(t) - b * st * sin(t);
		double y = a * st * cos(t) + b * ct * sin(t);
		double xp = a * ct * (-sin(t)) - b * st * cos(t); // derivative of x
		double yp = a * st * (-sin(t)) + b * ct * cos(t); // derivative of y
		//double ori = atan2(yp,xp)*180.0/M_PI + 180.0; // [0...360]
		Point2d n(yp, -xp);
		double nori = atan2(n.y, n.x) * 180.0 / M_PI + 180.0; // [0...360]
		double reg = 1. / norm(n);
		n = n * reg;
		Point2d normals[2];
		normals[0] = n;
		normals[1] = -n;
		Point2d p(x + center.x, y + center.y);
		cv::Scalar color = Scalar(0, 255, 0);
		color = Scalar(apply_heatmap(m_colors, 360, nori));
		//cout << ori << ": " << color << endl;
		if (draw || debug) {
			circle(outputImage, p, 3, color, -1, 1);
			//cout << "drawing " << nori << " in " << color << endl;
		}
		//line(outputImg, p, p1+n*20, Scalar(0,255,0),2,8);

		//circle(outputImage, p, 3, Scalar(255,0,0),-1,1);

		Point2d pn1 = p + normals[0] * search_length_1d; // 1D search length along normal
		Point2d pn2 = p + normals[1] * search_length_1d;
		//line(outputImage, pn1, pn2, Scalar(255,0,0),2,8);
		vector<cv::LineIterator> its;
		its.push_back(LineIterator(cannyImg, p, pn1));
		its.push_back(LineIterator(cannyImg, p, pn2));
		int matching = -1;
		// search along normals
		for (int k = 0; k < its[0].count && k < its[1].count;
				++k, ++its[0], ++its[1]) {
			// compare normal/gradient direction
			if (test_angle(ori, angle_thresh, rect, its[0], outputImage,
					nori)) {
				matching = 0;
				break;
			}
			if (test_angle(ori, angle_thresh, rect, its[1], outputImage,
					nori)) {
				matching = 1;
				break;
			}
		}
		if (matching >= 0) // found a matching point
				{
			++n_matched_points;
			Correspondence c;
			Point2d pp = its[matching].pos();
			c.point2d_to_point2d = pair<Point2d, Point2d>(p, pp);
			c.residual = cv::norm(pp - p);
			rbar2 += c.residual;
			if (m_use_beam_model_likelihood)
				log_likelihood += log(
						matching_likelihood(c.residual, search_length_1d));
			c.normal = normals[matching];
			correspondences.push_back(c);

			if (draw) {
				float o = ori.at<float>(pp.y, pp.x); // -M_PI
				float oc = o - nori; // between 360 and -360
				oc = fabs(oc); // between 0 and 360
				oc = fabs(180 - oc); //between 0 and 180;
				if (oc > 90)
					oc = 180 - oc; // between 0 and 90
				//cout << "pt: " << p << " yp: " << yp << " xp:" << xp << " nori: " << nori << " o: " << o << " cmp: " << oc << endl;;
			}

		} else // no match found, add penalty
		{
			rbar2 += search_length_1d;
			if (m_use_beam_model_likelihood)
				log_likelihood += log(
						matching_likelihood(search_length_1d,
								search_length_1d));
			else
				rbar2 += search_length_1d;
			if (draw || debug)
				line(outputImage, pn1, pn2, Scalar(0, 0, 255), 1, 8); // red

		}

	}
	rbar2 /= n_sampled_points;

	// draw
	if (draw || debug) {
		for (uint ci = 0; ci < correspondences.size(); ++ci) {
			Correspondence & c = correspondences[ci];
			Point2d & p1 = c.point2d_to_point2d.first;
			Point2d & p2 = c.point2d_to_point2d.second;
			line(outputImage, p1, p2, Scalar(0, 255, 0), 1, 8); // green
			//line(outputImage, c.point2d_to_point2d.first, c.point2d_to_point2d.second, Scalar(255,0,0),1,8);
		}

		if (draw) {
			ROS_INFO("Managed to match %zu / %d sample points",
					correspondences.size(), n_sampled_points);
			cout << "[";
			for (uint ci = 0; ci < correspondences.size(); ++ci) {
				cout << correspondences[ci].point2d_to_point2d.first;
				cout << "; ";
			}
			cout << "]" << endl;
			cout << "[";
			for (uint ci = 0; ci < correspondences.size(); ++ci) {
				cout << correspondences[ci].point2d_to_point2d.second;
				cout << "; ";
			}
			cout << "]" << endl;
		}
	}

	double lambda_nu = 0.5;
	double lambda_r = m_lambda_r;
	double l1 = 0.5;
	//double l1 = compute_likelihood_complete(cannyImg, ori, pose, outputImage);
	double l2 = exp(-lambda_r * rbar2);
	if (m_use_beam_model_likelihood)
		likelihood = (log_likelihood);
	else
		likelihood = log(l2);
	residual = rbar2;

	assert(exp(likelihood) > 0.0);
    if (std::isnan(likelihood)) {
		ROS_WARN("likelihood is nan");
	}
    assert(!std::isnan(likelihood));

	return true;
}

inline double LikelihoodCircleMeasurementEdge::matching_likelihood(double res,
		double max_range) {

	double p = 0.0;
	if (res < max_range) {
		p = m_zHit / (SQRT_2_PI * m_sigmaHit)
				* exp(-0.5 * (res * res) / (m_sigmaHit * m_sigmaHit));
		p += m_zRand / max_range;
	} else
		p = m_zMax;

	assert(p > 0.0);
	return p;
}

void sphere_to_quadric(const Eigen::Vector3d & pos,
		const Eigen::Matrix<double, 3, 4> & projection_matrix,
		double sphere_radius, Eigen::Matrix3d & Q) {
	double gamma = pos.transpose() * pos - sphere_radius * sphere_radius;
	double beta = 1.0 / (pos.transpose() * pos - gamma);
	Eigen::Matrix4d Sstar;
	Sstar << Eigen::Matrix3d::Identity() - beta * pos * pos.transpose(), -beta
			* pos, -beta * pos.transpose(), -beta;
	Eigen::Matrix3d Cstar = projection_matrix * Sstar
			* projection_matrix.transpose();
	Q = Cstar.inverse();
}

void quadric_to_sphere(const Eigen::Matrix3d & Q,
		const Eigen::Matrix<double, 3, 4> & projection_matrix,
		double sphere_radius, Eigen::Vector3d & a) {
	Eigen::Matrix3d Cstar = Q.inverse();
	Eigen::Matrix3d A = projection_matrix.block(0, 0, 3, 3);
	Eigen::Matrix3d vv = A * A.transpose() - Cstar;
	Eigen::Vector3d v;
	v << sqrt(vv(0, 0)), sqrt(vv(1, 1)), sqrt(vv(2, 2));
	a = A.inverse() * (sphere_radius * v);
}

void tf_to_opencv(const tf::Transform &pose, cv::Mat_<double> & R3,
		cv::Mat_<double> & T3) {
	double rot3x3_buffer[9];
	cv::Mat_<double> rot3x3(3, 3, rot3x3_buffer);
	T3(0, 0) = pose.getOrigin().x();
	T3(1, 0) = pose.getOrigin().y();
	T3(2, 0) = pose.getOrigin().z();
	// Convert to Rodrigues rotation
	const tf::Matrix3x3 &basis = pose.getBasis();
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			rot3x3(i, j) = basis[i][j];
	cv::Rodrigues(rot3x3, R3);

}

bool ellipse_from_quadric(const Eigen::Matrix3d & CstarI, Vector5d & z,
		Eigen::Matrix2d & U) {
	double av = CstarI(0, 0);
	double bv = CstarI(0, 1);
	double cv = CstarI(0, 2);
	double dv = CstarI(1, 1);
	double ev = CstarI(1, 2);
	double fv = CstarI(2, 2);
	Eigen::Matrix2d A;
	A << av, bv, bv, dv;
	Eigen::Vector2d B;
	B << 2 * cv, 2 * ev;
	if (av * dv - bv * bv <= 0) {
		//cerr << boost::format("This is not an ellipse at (%6.3f, %6.3f, %6.3f") % pos(0) % pos(1) % pos(2) << endl;
		cerr << boost::format("This is not an ellipse at") << endl;
		//throw std::runtime_error("This is not an ellipse");
		return false;
	}

	Eigen::Vector2d K2;
	double b1 = B(0);
	double b2 = B(1);
	double a11 = av;
	double a12 = bv;
	double a22 = dv;
	double denom = 2 * (a12 * a12 - a11 * a22);
	K2 << (a22 * b1 - a12 * b2) / denom, (a11 * b2 - a12 * b1) / denom;
	double mu = 1
			/ (a11 * K2(0) * K2(0) + 2 * a12 * K2(0) * K2(1)
					+ a22 * K2(1) * K2(1) - fv);
	double m11 = mu * a11;
	double m12 = mu * a12;
	double m22 = mu * a22;
	double la1 = ((m11 + m22) + sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0; // larger
	double la2 = ((m11 + m22) - sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0; // smaller
	double b = 1 / sqrt(la1); // semi-minor axis length (shorter)
	double a = 1 / sqrt(la2); // semi-major axis length (longer)
	Eigen::Vector2d U1, U2; //direction of minor and major axis
	if (m11 >= m22) {
		U1 << (la1 - m22), m12;
		U1 = U1 / U1.norm(); // minor axis
	} else {
		U1 << m12, (la1 - m11);
		U1 = U1 / U1.norm(); // minor axis
	}
	U2 << -U1(1), U1(0); //U2 (major axis) is orthogonal to U1 (right hand: thumb=U1, index finger=U2) --> 90 deg rotation around z-axis
	double theta2 = 0.5 * atan2(-2 * a12, (a22 - a11)); // angle between major axis (U2) and pos. x-axis
	z << K2(0), K2(1), a, b, theta2; // (b*U1), (a*U2) are the minor/major semi axis
	// TODO: this makes more sense to me, but is it correct?
	U2 = -U2;
	U(0, 0) = U1(0);
	U(0, 1) = U1(1);
	U(1, 0) = U2(0);
	U(1, 1) = U2(1);
	return true;
}

vector<cv::Vec3b> generate_heatmap(bool wrap) {
	vector<cv::Vec3b> colors;
	Vec3b red(0, 0, 255);
	Vec3b green(0, 255, 0);
	Vec3b blue(255, 0, 0);
	Vec3b yellow(0, 255, 255);
	Vec3b cyan(255, 255, 0);
	Vec3b magenta(255, 0, 255);
	/*
	 colors.push_back( blue );
	 colors.push_back( green );
	 //colors.push_back( yellow);
	 colors.push_back( red );
	 */
	colors.push_back(red);
	colors.push_back(cyan);
	colors.push_back(green);
	colors.push_back(yellow);

	if (wrap)
		colors.push_back(colors[0]);

	return colors;
}

cv::Vec3b apply_heatmap(const std::vector<cv::Vec3b> & colors, int max_val,
		double val) {
	cv::Vec3b color(0, 0, 0);
	int ncolors = colors.size() - 1; // 2 / 3
	double t = val / max_val;
	if (t < 0 || t > 1)
		return (color);
	int color_idx_lower = int(t * ncolors);
	if (color_idx_lower < 0 || color_idx_lower + 1 >= (int) colors.size()) {
		cerr << "This should not happen! color out of range" << endl;
		return (color);
	}
	double c = (t - (double) color_idx_lower / ncolors) * ncolors;
	color = (1 - c) * colors[color_idx_lower] + c * colors[color_idx_lower + 1];

	return (color);
}

bool test_angle(const cv::Mat & ori, float angle_thresh, Rect & rect,
		LineIterator & it, cv::Mat & outputImage, double line_orientation) {
	Scalar red(0, 0, 255);
	Scalar cyan(255, 255, 0);
	Scalar green(0, 255, 0);
	Scalar yellow(0, 255, 255);

	Point2f p = it.pos();
	if (!rect.contains(p))
		return false;

	uchar * val = *it; // assume binary image, check if occupied
	if ((int) *val <= 127)
		return false;

	float o = ori.at<float>(p.y, p.x); // -M_PI
	o = o - line_orientation; // between 360 and -360
	o = fabs(o); // between 0 and 360
	o = fabs(180 - o); //between 0 and 180;
	if (o > 90)
		o = 180 - o; // between 0 and 90
	if (o < angle_thresh)
		return true;
	return false;
}

cv::Mat orientationMap(const cv::Mat& mag, const cv::Mat& ori, double thresh,
		const std::vector<Vec3b> & colors) {
	Mat oriMap = Mat::zeros(ori.size(), CV_8UC3);
	Vec3b red(0, 0, 255);
	Vec3b cyan(255, 255, 0);
	Vec3b green(0, 255, 0);
	Vec3b yellow(0, 255, 255);
	for (int i = 0; i < mag.rows * mag.cols; i++) {
		float* magPixel = reinterpret_cast<float*>(mag.data + i * sizeof(float));
		if (*magPixel > thresh) {
			float* oriPixel = reinterpret_cast<float*>(ori.data
					+ i * sizeof(float));
			Vec3b* mapPixel = reinterpret_cast<Vec3b*>(oriMap.data
					+ i * 3 * sizeof(char));
			*mapPixel = apply_heatmap(colors, 360, *oriPixel);
		}
	}

	return oriMap;
}


} /* namespace kinematic_calibration */

