#ifndef __MarkerWorldCoornidate_h__
#define __MarkerWorldCoornidate_h__

#include <opencv2/opencv.hpp>
#include <vector>

#define MARKER_ROW_NUM 6
#define MARKERS_ROW_DISTANCE 60.0	//cm
#define MARKERS_COL_DISTANCE 50.0	//cm

class MarkerWorld
{
public:
	MarkerWorld();
	MarkerWorld(int _id, cv::Point3f _coordinate);
	MarkerWorld(int _id, float _x, float _y, float _z);
	~MarkerWorld();

	cv::Point3f coordinate;

	int id;

private:

};


class MarkerWorldCoordinate
{
public:
	MarkerWorldCoordinate();
	MarkerWorldCoordinate(size_t _size);
	~MarkerWorldCoordinate();

	size_t size();
	bool setCoordinate(MarkerWorld mw);
	cv::Point3f getCoordinate(int _id);

private:
	size_t m_size;
	std::vector<MarkerWorld> coorTable;
};

#endif // __MarkerWorldCoornidate_h__
