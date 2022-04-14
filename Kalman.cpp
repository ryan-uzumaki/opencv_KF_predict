﻿#include "opencv2/opencv.hpp"
#include "Kalman.h"
#include <iostream>

using namespace std;
using namespace cv;

Kalman::Kalman() :kf(4, 2, 0) {
	cv::setIdentity(kf.measurementMatrix, cv::Scalar::all(1));  // ²âÁ¿ŸØÕó H
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-3));  // ²âÁ¿ÔëÉùŸØÕó R
	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));  // ŽŠÀíÔëÉùŸØÕó Q
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));  // ×îÐ¡Ð­·œ²îŸØÕó p_k

	float A[] = { 1, 0, _t, 0,
		0, 1, 0, _t,
		0, 0, 1, 0,
		0, 0, 0, 1 };
	kf.transitionMatrix = cv::Mat(4, 4, CV_32F, A).clone();  // ×ŽÌ¬×ªÒÆŸØÕóA
	kf.statePost = (cv::Mat_<float>(4, 1) << _rect_lu.x, _rect_lu.y, 0, 0);  // ×ŽÌ¬Öµ³õÊŒ»¯ X_k
	measurement = cv::Mat::zeros(2, 1, CV_32F);  // ²âÁ¿Öµ³õÊŒ»¯ X_k
}

Point2f Kalman::update() {
	cv::Mat prediction = kf.predict();  // Ô€²Ä1�7
	cv::Point2f predict_pt = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));

	// ÉèÖÃ¹Û²âÖµ
	//measurement.at<float>(0, 0) = _vx * _t + _rect_lu.x;
	//measurement.at<float>(1, 0) = _vy * _t + _rect_lu.y;
	measurement.at<float>(0, 0) = _rect_lu.x;
	measurement.at<float>(1, 0) = _rect_lu.y;

	kf.correct(measurement);

	return predict_pt;
}

void Kalman::setRectinfo(cv::Rect& preRect) {
	_rect_lu = static_cast<cv::Point2f>(preRect.tl());
	_rect_width = preRect.width;
	_rect_height = preRect.height;

}

void Kalman::setFrameTime(double t) {
	_t = t;
	//cout << "t : " << t << endl;
}

void Kalman::setSpeed(cv::Rect& last_rect, cv::Rect& rect) {
	double _vx = (rect.tl().x - last_rect.tl().x) / _t;
	double _vy = (rect.tl().y - last_rect.tl().y) / _t;
	//cout << "vx : " << _vx << "\n" << "vy : " << _vy << endl;
}