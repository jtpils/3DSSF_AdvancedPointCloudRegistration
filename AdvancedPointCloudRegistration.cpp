#include <iostream>
#include <fstream>
#include <random>
#include <time.h>
#include <windows.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <Eigen/Dense>
#include "FileReaderWriter.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef struct Trafo {
	Mat rot;
	Point3d offset;
} Trafo;

typedef struct Match {
	size_t index1;
	size_t index2;
	double distance;

	Match(size_t i1, size_t i2, double d) {
		index1 = i1;
		index2 = i2;
		distance = d;
	}

	Match() {
	}
};

Trafo registration(vector<Point3d>& points1, vector<Point3d>& points2, vector<Match>& matches) {
	Trafo ret;

	int NUM = matches.size();

	Point3d offset1(0.0, 0.0, 0.0);
	Point3d offset2(0.0, 0.0, 0.0);

	for (int i = 0; i < NUM; i++) {
		Point3d v1 = points1[matches[i].index1];
		Point3d v2 = points2[matches[i].index2];

		offset1 += v1;
		offset2 += v2;
	}

	offset1 /= NUM;
	offset2 /= NUM;

	Mat M1(3, NUM, CV_32F);
	Mat M2(3, NUM, CV_32F);

	Mat H = Mat::zeros(Size(3, 3), CV_32F);

	for (int i = 0; i < NUM; i++) {
		Point3d pt1 = points1[matches[i].index1] - offset1;
		Point3d pt2 = points2[matches[i].index2] - offset2;

		M1.at<float>(0, i) = pt1.x;
		M1.at<float>(1, i) = pt1.y;
		M1.at<float>(2, i) = pt1.z;
		M2.at<float>(0, i) = pt2.x;
		M2.at<float>(1, i) = pt2.y;
		M2.at<float>(2, i) = pt2.z;
	}

	H = M1 * M2.t();

	Mat w(3, 3, CV_32F);
	Mat u(3, 3, CV_32F);
	Mat vt(3, 3, CV_32F);

	SVD::compute(H, w, u, vt);

	Mat rot = vt.t() * u.t();

	double det = determinant(rot);

	if (det < 0) {
		vt.at<float>(2, 0) *= -1;
		vt.at<float>(2, 1) *= -1;
		vt.at<float>(2, 2) *= -1;
		rot = vt.t() * u.t();
	}

	ret.rot = rot;

	float numerator = 1.0;
	float denominator = 1.0;

	Mat c1(3, 1, CV_32F);
	Mat c2(3, 1, CV_32F);

	c1.at<float>(0, 0) = offset1.x;
	c1.at<float>(1, 0) = offset1.y;
	c1.at<float>(2, 0) = offset1.z;
						 
	c2.at<float>(0, 0) = offset2.x;
	c2.at<float>(1, 0) = offset2.y;
	c2.at<float>(2, 0) = offset2.z;

	Mat offset(3, 1, CV_32F);

	offset = -rot * c1 + c2;

	ret.offset.x = offset.at<float>(0, 0);
	ret.offset.y = offset.at<float>(1, 0);
	ret.offset.z = offset.at<float>(2, 0);

	return ret;
}

void Rotate(vector<Point3d>& points, double alpha, double beta, double gamma) {
	Matx33d R_x(1,				0,				0,
				0,				cos(alpha),	   -sin(alpha),
				0,				sin(alpha),		cos(alpha)
	);

	Matx33d R_y(cos(beta),		0,				sin(beta),
				0,				1,				0,
			   -sin(beta),		0,				cos(beta)
	);

	Matx33d R_z(cos(gamma),	   -sin(gamma),		0,
				sin(gamma),		cos(gamma),		0,
				0,				0,				1);

	Matx33d R = R_z * R_y * R_x;

	for (int i = 0; i < points.size(); ++i) {
		points.at(i) = R * points.at(i);
	}
}

void RotateRandom(vector<Point3d>& points) {
	double alpha = (rand() % 5) / 180.0 * CV_PI;
	double beta = (rand() % 5) / 180.0 * CV_PI;
	double gamma = (rand() % 5) / 180.0 * CV_PI;

	Rotate(points, alpha, beta, gamma);
}

void Translate(vector<Point3d>& points, double x, double y, double z) {
	for (int i = 0; i < points.size(); ++i) {
		points.at(i) += Point3d(x, y, z);
	}
}

void TranslateRandom(vector<Point3d>& points) {
	double x = (rand() % 10) - 5;
	double y = (rand() % 10) - 5;
	double z = (rand() % 10) - 5;

	Translate(points, x, y, z);
}

Point3d GetCenter(vector<Point3d>& points) {
	Point3d center = Point3d(0, 0, 0);
	for (int i = 0; i < points.size(); ++i) {
		center += points.at(i);
	}
	center = center / (double)points.size();
	return center;
}

void MoveToCenter(vector<Point3d>& points) {
	Point3d center = GetCenter(points);
	for (int i = 0; i < points.size(); ++i) {
		points.at(i) -= center;
	}
}

void AddGaussianNoise(vector<Point3d>& points, double sigma = 1.0) {
	default_random_engine generator;
	normal_distribution<double> distribution(0, sigma);

	for (int i = 0; i < points.size(); ++i) {
		double randX = distribution(generator);
		double randY = distribution(generator);
		double randZ = distribution(generator);

		points.at(i) += Point3d(randX, randY, randZ);
	}
}

bool isRotationMatrix(Mat& R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;
}

Vec3d rotationMatrixToEulerAngles(Mat& R)
{
	//assert(isRotationMatrix(R));

	float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

	bool singular = sy < 1e-6;

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
		y = atan2(-R.at<float>(2, 0), sy);
		z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
	}
	else
	{
		x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
		y = atan2(-R.at<float>(2, 0), sy);
		z = 0;
	}
	return Vec3d(x, y, z);
}

int main(int argc, char* argv[])
{
	string file1 = argv[1];
	string file2 = argv[2];

	FileReaderWriter frw;
	vector<Point3d> points1_ = frw.ReadPLY(file1);
	vector<Point3d> points2_ = frw.ReadPLY(file2);

	double alphaT = CV_PI / 9;
	double betaT = CV_PI / 18;
	double gammaT = -CV_PI / 12;
	
	double xT = 2;
	double yT = -3;
	double zT = 1;

	Point3d center = GetCenter(points1_);
	Translate(points1_, -center.x, -center.y, -center.z);
	Rotate(points1_, alphaT, betaT, gammaT);
	Translate(points1_, center.x + xT, center.y + yT, center.z + zT);

	if (points1_.size() > points2_.size()) {
		vector<Point3d> pointsTemp = points1_;
		points1_ = points2_;
		points2_ = pointsTemp;
	}

	frw.WriteXYZ("points1.xyz", points1_);
	frw.WriteXYZ("points2.xyz", points2_);

	vector<float> sigmas = { 0.01, 0.05, 0.1, 0.25 };
	vector<float> trims = { 0.1, 0.25, 0.5, 1.0 };

	for (int sigma_num = 0; sigma_num < sigmas.size(); ++sigma_num) {
		for (int trim_num = 0; trim_num < trims.size(); ++trim_num) {
			string folder = "sigma_" + to_string(sigmas[sigma_num]) + "-trim_" + to_string(trims[trim_num]);
			CreateDirectoryA(folder.c_str(), NULL);

			ofstream conf(folder + "\\configuration.txt");
			conf << alphaT << endl;
			conf << betaT << endl;
			conf << gammaT << endl;
			conf << xT << endl;
			conf << yT << endl;
			conf << zT << endl;
			ofstream MSEF(folder + "\\MSE.txt");
			ofstream rotF(folder + "\\rotation.txt");
			ofstream traF(folder + "\\translation.txt");

			vector<Point3d> points1 = points1_;
			vector<Point3d> points2 = points2_;

			AddGaussianNoise(points1, sigmas[sigma_num]);
			AddGaussianNoise(points2, sigmas[sigma_num]);

			frw.WriteXYZ(folder + "\\points1.xyz", points1);
			frw.WriteXYZ(folder + "\\points2.xyz", points2);

			clock_t tStart = clock();

			const size_t M = points1.size();
			const size_t N = points2.size();
			const size_t dim = 3;

			cv::Mat_<float> features(0, 3);

			for (Point3d point : points2) {
				//Fill matrix
				cv::Mat row = (cv::Mat_<float>(1, 3) << point.x, point.y, point.z);
				features.push_back(row);
			}

			cv::flann::Index flann_index(features, cv::flann::KDTreeIndexParams(1));

			unsigned int max_neighbours = 1;

			vector<Match> matches;

			int max_iterations = 100;
			float epsilon = 0.0000001;

			vector<double> MSEs;
			vector<double> alphas;
			vector<double> betas;
			vector<double> gammas;
			vector<double> xs;
			vector<double> ys;
			vector<double> zs;

			Mat fullR = Mat::eye(3, 3, CV_32F);
			Point3d fullT = Point3d(0, 0, 0);

			int iter = 0;
			do {
				for (int i = 0; i < M; ++i) {
					cv::Mat query = (cv::Mat_<float>(1, 3) << points1[i].x, points1[i].y, points1[i].z);
					cv::Mat indices, dists;

					flann_index.knnSearch(query, indices, dists, max_neighbours, cv::flann::SearchParams(32));

					matches.push_back(Match(i, indices.at<int>(0), dists.at<float>(0)));
				}

				sort(matches.begin(), matches.end(), [](auto const& a, auto const& b) { return a.distance < b.distance; });
				int trimmedSize = int(trims[trim_num] * matches.size());
				matches.resize(trimmedSize);
				float MSE = 0;
				for (int i = 0; i < trimmedSize; ++i) {
					MSE += matches.at(i).distance;
				}
				MSE /= trimmedSize;
				cout << "MSE: " << MSE << endl;
				MSEs.push_back(MSE);
				MSEF << MSE << endl;

				Trafo trafo = registration(points1, points2, matches);
				matches.clear();

				fullR = trafo.rot * fullR;

				Vec3d angles = rotationMatrixToEulerAngles(fullR);
				alphas.push_back(angles[0]);
				betas.push_back(angles[1]);
				gammas.push_back(angles[2]);
				rotF << iter << " " << alphaT - angles[0] << " " << betaT - angles[1] << " " << gammaT - angles[2] << " " << angles[0] << " " << angles[1] << " " << angles[2] << endl;

				fullT += trafo.offset;
				xs.push_back(fullT.x);
				ys.push_back(fullT.y);
				zs.push_back(fullT.z);
				traF << iter << " " << xT - fullT.x << " " << yT - fullT.y << " " << zT - fullT.z << " " << fullT.x << " " << fullT.y << " " << fullT.z << endl;

				//Write result
				stringstream ss;
				ss << folder << "\\iter" << iter << ".xyz";

				for (int i = 0; i < M; ++i) {
					Mat p1(3, 1, CV_32F);
					p1.at<float>(0, 0) = points1[i].x;
					p1.at<float>(1, 0) = points1[i].y;
					p1.at<float>(2, 0) = points1[i].z;

					p1 = (trafo.rot * p1);

					points1[i].x = p1.at<float>(0, 0);
					points1[i].y = p1.at<float>(1, 0);
					points1[i].z = p1.at<float>(2, 0);

					points1[i] += trafo.offset;
				}

				frw.WriteXYZ(ss.str(), points1);
				
				cout << ss.str() << endl;

				if (iter > 0) {
					if (abs(MSEs.at(iter) - MSEs.at(iter - 1)) < epsilon) {
						break;
					}
				}
				iter++;
			} while (iter < max_iterations);

			double time = (double)(clock() - tStart) / CLOCKS_PER_SEC;

			conf << time;
		}
	}

	return 0;
}