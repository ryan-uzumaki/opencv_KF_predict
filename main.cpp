#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

typedef struct ThreshHSV {
	int rh_h = 10;
	int rh_l = 0;
	int rh_h2 = 180;
	int rh_l2 = 160;
}ThreshHSV;

class Kalman
{
private:
	cv::KalmanFilter kf;
	cv::Mat measurement;

	cv::Point2f _rect_lu;
	double _rect_width;
	double _rect_height;

	double _vx;
	double _vy;
	double _t;

public:
	Kalman() :kf(4, 2, 0) {
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
	// update
	cv::Point2f update() {
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

	void setRectinfo(cv::Rect& preRect) {
		_rect_lu = static_cast<cv::Point2f>(preRect.tl());
		_rect_width = preRect.width;
		_rect_height = preRect.height;

	};

	void setFrameTime(double t) {
		_t = t;
		//cout << "t : " << t << endl;
	}

	void setSpeed(cv::Rect& last_rect, cv::Rect& rect) {
		double _vx = (rect.tl().x - last_rect.tl().x) / _t;
		double _vy = (rect.tl().y - last_rect.tl().y) / _t;
		//cout << "vx : " << _vx << "\n" << "vy : " << _vy << endl;
	}

};

double get_distance(int W, int P) {
	double F = 550;
	double D = 0;
	D = (W * F) / P;
	return D;
}

string Convert(float Num)
{
	std::ostringstream oss;
	oss << Num;
	std::string str(oss.str());
	return str;
}

int main()
{
	VideoCapture capture(1);


	Kalman kalman;
	cv::Mat image = cv::Mat::zeros(Size(800, 600), CV_8UC3);
	cv::Rect last_rect;
	while (true) {
		//double t = (double)cv::getTickCount();

		capture >> image;
		if (image.empty()) {
			printf("Error! The frame is empty!\n");
			break;
		}

		int known_W = 9;
		int known_P = 510;
		Mat temp = Mat::zeros(image.size(), image.type());
		Mat m = Mat::zeros(image.size(), image.type());
		addWeighted(image, 0.24, m, 0.0, 0, temp);
		Mat dst;
		bilateralFilter(temp, dst, 5, 20, 20);
		Mat m_ResImg;
		cvtColor(dst, m_ResImg, COLOR_BGR2HSV);
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		Mat dstImage;
		inRange(m_ResImg, Scalar(100, 43, 46), Scalar(124, 255, 255), dstImage);

		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(dstImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
		cv::Rect rect, predict_rect;
		cv::Point2f predict_pt;
		for (size_t i = 0; i < contours.size(); ++i) {
			double contour_area = cv::contourArea(contours[i]);
			if (contour_area < 100) continue;
			rect = cv::boundingRect(contours[i]);
			//if (rect.height < 50) continue;
			kalman.setRectinfo(rect);
		}
		// vector<double> area;
		// for (size_t i = 0; i < contours.size(); ++i) {
		// 	area.push_back(contourArea(contours[i]));
		// }
		// int maxIndex = max_element(area.begin(), area.end()) - area.begin();
		// Rect ret_1 = boundingRect(contours[maxIndex]);
		// // rect = cv::boundingRect(contours[i]);
		// // //if (rect.height < 50) continue;
		// // kalman.setRectinfo(rect);
		// for (size_t i=0;i <contours.size(); i++){
		// 	kalman.setRectinfo(ret_1);
		// }
		predict_pt = kalman.update();

		//cv::circle(image, predict_pt, 5, cv::Scalar(0, 255, 0), -1, 8);
		predict_rect = cv::Rect(predict_pt.x, predict_pt.y, rect.width, rect.height);

		//if (rect.empty()) {
		//	kalman.setRectinfo(last_rect);
		//	cout << predict_rect.size().width << endl;
		//	predict_pt = kalman.update();
		//	predict_rect = cv::Rect(predict_pt.x, predict_pt.y, predict_rect.width, rect.height);
		//}

		const float scale = 1;
		cv::Point rev_pt;
		if ((((rect.tl().x + scale * (rect.tl().x - predict_pt.x)) <= 800) && ((rect.tl().x + scale * (rect.tl().x - predict_pt.x)) >= 0) &&
			((rect.tl().y + scale * (rect.tl().y - predict_pt.y)) <= 600) && ((rect.tl().y + scale * (rect.tl().y - predict_pt.y)) >= 0))) {
			if ((fabs(rect.tl().x - predict_pt.x) > 3) || (fabs(rect.tl().y - predict_pt.y) > 3)) {
				rev_pt.x = rect.tl().x + scale * (rect.tl().x - predict_pt.x);
				rev_pt.y = rect.tl().y + scale * (rect.tl().y - predict_pt.y);
			}
			else {
				rev_pt.x = rect.tl().x;
				rev_pt.y = rect.tl().y;
			}
		}
		else {
			rev_pt.x = rect.tl().x;
			rev_pt.y = rect.tl().y;
		}

		predict_rect = cv::Rect(rev_pt.x, rev_pt.y, rect.width, rect.height);
		//cv::circle(image, predict_rect.tl(), 5, cv::Scalar(0, 255, 0), -1, 8);
		cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 3, 8);
		cv::rectangle(image, predict_rect, cv::Scalar(0, 255, 0), 2, 8);
		double dist = get_distance(known_W, rect.width);
		string dist_str = Convert(dist);
		putText(image, "Distance:" + dist_str + "cm", Point(50, 50), FONT_HERSHEY_COMPLEX, 1, Scalar(50, 250, 50), 2, 8);
		cv::imshow("detected", image);
		int c = waitKey(10);
		if (c == 27) { // ÍË³ö
			break;
		}
	}
	capture.release();
	cv::destroyAllWindows();

	return 0;
}