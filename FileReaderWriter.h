#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class FileReaderWriter
{
private:
public:
	static vector<Point3d> ReadPLY(string fileName);
	static void WriteXYZ(string fileName, vector<Point3d> &points, string header = "");
};