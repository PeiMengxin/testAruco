/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/

#pragma warning (disable : 4290)

#include <iostream>
#include "aruco\aruco.h"
#include "aruco\cvdrawingutils.h"
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"

extern MarkerWorld CoordinateTable[];

using namespace std;
using namespace cv;
using namespace aruco;

#define TRANS_WORLD 1
#define WRITE_VIDEO  0

#define MED_WIDTH_NUM 50
#define MED_FIL_ITEM  20

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
float med_filter_out[MED_FIL_ITEM];

int med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(int item, int width_num, float in)
{
	int i, j;
	float t;
	float tmp[MED_WIDTH_NUM];

	if (item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM)
	{
		return 0;
	}
	else
	{
		if (++med_fil_cnt[item] >= width_num)
		{
			med_fil_cnt[item] = 0;
		}

		med_filter_tmp[item][med_fil_cnt[item]] = in;

		for (i = 0; i < width_num; i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}

		for (i = 0; i < width_num - 1; i++)
		{
			for (j = 0; j<(width_num - 1 - i); j++)
			{
				if (tmp[j] > tmp[j + 1])
				{
					t = tmp[j];
					tmp[j] = tmp[j + 1];
					tmp[j + 1] = t;
				}
			}
		}


		return (tmp[(int)width_num / 2]);
	}
}

void initTraj(cv::Mat &traj, float rectsize, float offset);

void initTraj(cv::Mat &traj, float rectsize, float offset)
{
	Point origin(offset, offset);
	for (size_t i = 0; i < 36; i++)
	{
		Rect rect((CoordinateTable[i].coordinate.x - rectsize / 2) * 2 + origin.x, (CoordinateTable[i].coordinate.y - rectsize / 2) * 2 + origin.y, rectsize * 2, rectsize * 2);

		rectangle(traj, rect, Scalar(0, 0, 0), 2);
	}

	cv::line(traj, origin, Point(origin.x, 690), Scalar(72, 61, 139), 2);
	cv::line(traj, origin, Point(690, origin.y), Scalar(72, 61, 139), 2);

	cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y - 10), Scalar(72, 61, 139), 2);
	cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y + 10), Scalar(72, 61, 139), 2);

	cv::line(traj, Point(origin.x, 690), Point(origin.x - 10, 690 - 10), Scalar(72, 61, 139), 2);
	cv::line(traj, Point(origin.x, 690), Point(origin.x + 10, 690 - 10), Scalar(72, 61, 139), 2);
}

int main(int argc, char **argv)
{
	try
	{
		string cameraParamFileName1("out_camera_data1.yml");
		string cameraParamFileName("PS3.yml");

		FileStorage fs;
		fs.open(cameraParamFileName1, FileStorage::READ);

		Mat intrinsic_mat, distortion_coeffs;
		fs["camera_matrix"] >> intrinsic_mat;
		fs["distortion_coefficients"] >> distortion_coeffs;

		cout << intrinsic_mat << endl;
		cout << distortion_coeffs << endl;

		aruco::CameraParameters CamParam;
		MarkerDetector MDetector;
		vector< Marker > Markers;

		// read the input image
		cv::Mat InImage;
		// try opening first as video
		VideoCapture cap(0);

		if (cap.isOpened()) {
			cap.grab();
			cap.retrieve(InImage);
		}
		else {
			InImage = cv::imread(argv[1]);
		}
		// at this point, we should have the image in InImage
		// if empty, exit
		if (InImage.total() == 0) {
			cerr << "Could not open input" << endl;
			return 0;
		}

		//read camera parameters if specifed
		CamParam.readFromXMLFile(cameraParamFileName);
		// resizes the parameters to fit the size of the input image
		CamParam.resize(InImage.size());

		cout << CamParam.CameraMatrix << endl;
		cout << CamParam.Distorsion << endl;

		float MarkerSize = 20;

		cv::namedWindow("thes", 1);

		int p1 = 7;
		int p2 = 7;
		int t_p_range = 2;
		createTrackbar("p1", "thes", &p1, 101);
		createTrackbar("p2", "thes", &p2, 50);
		createTrackbar("range", "thes", &t_p_range, 31);

		ostringstream ostr_pos;
		ostringstream ostr_angle;

#if WRITE_VIDEO

		VideoWriter videowriter;
		videowriter.open("video.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30, InImage.size());

#endif

		double pos[3] = { 0 };
		Point3f pos_camera(0, 0, 0);
		Attitude attitude_camera;
		Mat traj(720, 720, CV_8UC3, Scalar::all(255));

		float offset = 50;

		initTraj(traj, MarkerSize, offset);

		Mat traj_empty = traj.clone();

		TickMeter tm;
		tm.reset();

		Mat srcImg;

		int drawPointSize = 50;
		vector<Point2f> drawPointKF;
		vector<Point2f> drawPointSrc;
		drawPointKF.resize(drawPointSize,Point2f(0,0));
		drawPointSrc.resize(drawPointSize, Point2f(0, 0));

#define USE_CANNY 0
#if USE_CANNY 
		MDetector.setThresholdMethod(MarkerDetector::CANNY);
#endif

#define USE_KF 1
#if USE_KF
		RNG rng;
		//1.kalman filter setup
		const int stateNum = 4;                                      //状态值4×1向量(x,y,△x,△y)
		const int measureNum = 2;                                    //测量值2×1向量(x,y)	
		KalmanFilter KF(stateNum, measureNum, 0);

		KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A
		setIdentity(KF.measurementMatrix);                                             //测量矩阵H
		setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
		setIdentity(KF.measurementNoiseCov, Scalar::all(0.01));                        //测量噪声方差矩阵R
		setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
		rng.fill(KF.statePost, RNG::UNIFORM, 0, 360);									//初始状态值x(0)
		Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义

		cout << "KF" << KF.measurementMatrix << endl;
		cout << "KF" << KF.statePost << endl;
#endif

		while (true)
		{
			p1 = p1 / 2 * 2 + 1;
			p2 = p2 / 2 * 2 + 1;
			MDetector.setThresholdParamRange(t_p_range);
			MDetector.setThresholdParams(p1, p2);

			tm.reset();
			tm.start();
#define CAL 0  //0-------------------------------0V0--------------------------------------------------0
#if CAL
			cap >> srcImg;
			imshow("sb", srcImg);
			undistort(srcImg, InImage, intrinsic_mat, distortion_coeffs);
#else
			cap >> InImage;
#endif
			tm.stop();

			//cout << "capImg:" << tm.getTimeMilli() << endl;

#if WRITE_VIDEO
			videowriter << InImage;
#endif

			tm.reset();
			tm.start();

			// Ok, let's detect
			MDetector.detect(InImage, Markers, CamParam, MarkerSize);
			// for each marker, draw info and its boundaries in the image
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
			}

			if (CamParam.isValid() && MarkerSize != -1)
			{
				for (unsigned int i = 0; i < Markers.size(); i++)
				{
					//CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
					CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);

					//circle(InImage, Point(InImage.cols / 2, InImage.rows / 2), 3, CV_RGB(0, 0, 255), -1);

					getAttitude(Markers[i], attitude_camera);

					ostr_angle.clear();
					ostr_angle.str("");
					ostr_angle << "          Pit=" << (int)attitude_camera.Pit << " " << "Yaw=" << (int)attitude_camera.Yaw << " " << "Rol=" << (int)attitude_camera.Rol;

#if TRANS_WORLD
					getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_camera);

					ostr_pos.clear();
					ostr_pos.str("");
					ostr_pos << "          x=" << (int)pos_camera.x << " " << "y=" << (int)pos_camera.y << " " << "z=" << (int)pos_camera.z;
					putText(InImage, ostr_pos.str(), Markers[i].getCenter() - Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

#endif

					/*ostr_pos.clear();
					ostr_pos.str("");
					ostr_pos << "          x=" << (int)pos[0] << " " << "y=" << (int)pos[1] << " " << "z=" << (int)pos[2];

					putText(InImage, ostr_pos.str(), Markers[i].getCenter(), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);*/

					putText(InImage, ostr_angle.str(), Markers[i].getCenter() + Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

				}
			}
			cv::Point3f coordinate_camera(0, 0, 0);
			getCameraPosWithMarkers(Markers, coordinate_camera, 2);

			ostr_pos.clear();
			ostr_pos.str("");
			ostr_pos << "          x=" << (int)coordinate_camera.x << " " << "y=" << (int)coordinate_camera.y << " " << "z=" << (int)coordinate_camera.z;

			putText(InImage, ostr_pos.str(), Point(30, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

			static float x_show = 0, y_show = 0;

#if USE_KF
			//2.kalman prediction
			Mat prediction = KF.predict();
			Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')

			//3.update measurement
			if (coordinate_camera.x != 0 && coordinate_camera.y != 0)
			{
				measurement.at<float>(0) = (float)coordinate_camera.x;
				measurement.at<float>(1) = (float)coordinate_camera.y;
			}
			else
			{
				//KF.init(stateNum, measureNum, 0);
				KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A
				setIdentity(KF.measurementMatrix);                                             //测量矩阵H
				setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
				setIdentity(KF.measurementNoiseCov, Scalar::all(0.01));                        //测量噪声方差矩阵R
				setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
				rng.fill(KF.statePost, RNG::UNIFORM, 0, 360);									//初始状态值x(0)
				KF.statePost.at<float>(0) = (float)coordinate_camera.x;
				KF.statePost.at<float>(1) = (float)coordinate_camera.y;

				//measurement.at<float>(0) = (float)coordinate_camera.x;
				//measurement.at<float>(1) = (float)coordinate_camera.y;
			}


			//4.update
			KF.correct(measurement);

			//draw points;
			drawPointKF.insert(drawPointKF.begin(), predict_pt + predict_pt + Point(offset, offset));
			drawPointKF.pop_back();

			for (size_t i = 0; i < drawPointKF.size(); i++)
			{
				circle(traj, drawPointKF[i], 1, CV_RGB(0, 192, 192), 2);
			}
			//circle(traj, predict_pt + predict_pt + Point(offset, offset), 1, CV_RGB(0, 192, 192), 2);
#endif

			static float x = 0;
			static float y = 0;
			if ((int(coordinate_camera.x) != 0) && (int(coordinate_camera.y) != 0))
			{
				x = (coordinate_camera.x) * 2;
				y = (coordinate_camera.y) * 2;
			}
			static float x1, y1;
			float k = 0.1;
			if (x != 0 && y != 0)
			{
				Moving_Median(0, 20, x);
				Moving_Median(1, 20, y);
				x1 = x;// k*x1 + (1 - k)*x;
				y1 = y;// k*y1 + (1 - k)*y;
			}

			x_show = x;
			y_show = y;

			drawPointSrc.insert(drawPointSrc.begin(), Point(x_show, y_show) + Point(offset, offset));
			drawPointSrc.pop_back();

			for (size_t i = 0; i < drawPointSrc.size(); i++)
			{
				circle(traj, drawPointSrc[i], 8, CV_RGB(255, 0, 0), 1);
			}

			//circle(traj, Point(x_show, y_show)+Point(offset,offset), 8, CV_RGB(255, 0, 0), 1);

			imshow("Trajectory", traj);

			// show input with augmented information
			cv::imshow("in", InImage);
			// show also the internal image resulting from the threshold operation
			cv::imshow("thes", MDetector.getThresholdedImage());

			tm.stop();
			
			char c_key = cv::waitKey(10);
			if (c_key == 27) // wait for key to be pressed
			{
				break;
			}
			else if (c_key == 'c' || c_key == 'C')
			{
				traj = traj_empty.clone();
			}
			else if (c_key == 'd' || c_key == 'D')
			{
				cout << drawPointKF.size() << endl;
				cout << drawPointKF<< endl;
			}
			traj = traj_empty.clone();
	}

#if WRITE_VIDEO
		videowriter.release();
#endif
		return 0;
}
	catch (std::exception &ex)
	{
		cout << "Exception :" << ex.what() << endl;
	}
}
