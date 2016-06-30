#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include <vector>

using namespace std;
using namespace cv;
using namespace aruco;

extern MarkerWorld CoordinateTable[];

//自定义排序函数
bool SortByDown(const float &p1, const float &p2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
	return p1 > p2;//降序排列
}

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z)
{
	Mat R_X = Mat::eye(3, 3, CV_32FC1);
	Mat R_Y = Mat::eye(3, 3, CV_32FC1);
	Mat R_Z = Mat::eye(3, 3, CV_32FC1);

	alpha_X /= 57.3;
	alpha_Y /= 57.3;
	alpha_Z /= 57.3;

	R_X.at<float>(1, 1) = cos(alpha_X);
	R_X.at<float>(1, 2) = sin(alpha_X);
	R_X.at<float>(2, 1) = -sin(alpha_X);
	R_X.at<float>(2, 2) = cos(alpha_X);

	R_Y.at<float>(0, 0) = cos(alpha_Y);
	R_Y.at<float>(0, 2) = -sin(alpha_Y);
	R_Y.at<float>(2, 0) = sin(alpha_Y);
	R_Y.at<float>(2, 2) = cos(alpha_Y);

	R_Z.at<float>(0, 0) = cos(alpha_Z);
	R_Z.at<float>(0, 1) = sin(alpha_Z);
	R_Z.at<float>(1, 0) = -sin(alpha_Z);
	R_Z.at<float>(1, 1) = cos(alpha_Z);

	vector<Mat> dst;
	dst.push_back(R_X);
	dst.push_back(R_Y);
	dst.push_back(R_Z);

	return dst;

}

void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
	Mat Rot(3, 3, CV_32FC1);
	Rodrigues(Rvec, Rot);

	Rot = Rot.t();  // rotation of inverse
	Mat pos_camera = -Rot * Tvec; // translation of inverse

	pos.x = pos_camera.at<float>(0, 0);
	pos.y = -pos_camera.at<float>(1, 0);
	pos.z = pos_camera.at<float>(2, 0);
}

void getCameraPosWithMarkers(std::vector< aruco::Marker > Markers, cv::Point3f &pos_camera, int flag /*= 0*/)
{
	Point3f pos_world(0, 0, 0);

	switch (flag)
	{
		case 0:
		{
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);

				pos_camera += CoordinateTable[Markers[i].id - 1].coordinate + pos_world;

			}

			if (Markers.size()>0)
			{
				pos_camera.x = pos_camera.x / Markers.size();
				pos_camera.y = pos_camera.y / Markers.size();
				pos_camera.z = pos_camera.z / Markers.size();
			}

			break;
		}
		case 1:
		{
			float dis = 0;
			float dismin = 100000;

			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);

				dis = sqrt(pos_world.x*pos_world.x + pos_world.y*pos_world.y);

				if (dis<dismin)
				{
					dismin = dis;
					pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;
				}
			}
			break;
		}
		case 2:
		{
			std::vector<float> vec_x, vec_y;
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);
				pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;
				vec_x.push_back(pos_camera.x);
				vec_y.push_back(pos_camera.y);
			}
			if (vec_x.size()>0)
			{
				sort(vec_x.begin(), vec_x.end(), SortByDown);
				sort(vec_y.begin(), vec_y.end(), SortByDown);

				pos_camera.x = vec_x[vec_x.size() / 2];
				pos_camera.y = vec_y[vec_y.size() / 2];
			}
			

			break;
		}
			
		default:
			break;
	}
	
}

void getAttitude(aruco::Marker marker, Attitude &attitude)
{
	double pos[3] = { 0 };
	double ori[4] = { 0 };

	double q0, q1, q2, q3;

	marker.OgreGetPoseParameters(pos, ori);
	pos[0] = -pos[0];
	pos[1] = -pos[1];

	q0 = ori[0]; q1 = ori[1]; q2 = ori[2]; q3 = ori[3];

	attitude.Pit = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) *57.3f;
	attitude.Yaw = asin(2 * (q1 * q3 - q0 * q2)) *57.3f;
	attitude.Rol = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) *57.3f;
}
