#include "FileReaderWriter.h"
#include <fstream>

vector<Point3d> FileReaderWriter::ReadPLY(string fileName)
{
	vector<Point3d> points;

	ifstream file(fileName);
	if (file.is_open()) {
		string temp;
		int count = 0;
		do {
			file >> temp;
			if (temp == "element")
			{
				file >> temp;
				if (temp == "vertex")
					file >> count;
			}
		} while (temp != "end_header");
		
		double x, y, z, nx, ny, nz;
		for (int i = 0; i < count; ++i) {
			file >> x;
			file >> y;
			file >> z;
			file >> nx;
			file >> ny;
			file >> nz;
			points.push_back(Point3d(x, y, z));
		}
	}
	file.close();

	return points;
}

void FileReaderWriter::WriteXYZ(string fileName, vector<Point3d> &points, string header) {
	ofstream file(fileName);
	if (file.is_open()) {
		file << header;
		for (int i = 0; i < points.size(); ++i) {
			file << points.at(i).x << " " << points.at(i).y << " " << points.at(i).z << endl;
		}
	}
	file.close();
}