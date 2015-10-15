#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

//Physics problem, predicting path of a projectile object-----------------------------------------------------------
#define GRAVITY 9.81
#define PI 3.14159265
#define MAX_PREDICTION_POINTS 100

struct Point3D {
	double x;
	double y;
	double z;
	time_t timestamp;
	Point3D(double x_in = 0.0, double y_in = 0.0, double z_in = 0.0, double timestamp_in = 0.0) {
		x = x_in;
		y = y_in;
		z = z_in;
		timestamp = timestamp_in;
	}
};
struct Point3D initialPoint1;
struct Point3D initialPoint2;
double angleXY;
double angleXZ;
double initialV;

double angle_XY(struct Point3D &point1, struct Point3D &point2) {
	return atan2((point2.y - point1.y) , (point2.x - point1.x));
		//atan((point2.y - point1.y) / (point2.x - point1.x));
}
double angle_XZ(struct Point3D &point1, struct Point3D &point2) {
	return atan2((point2.z - point1.z) , (point2.x - point1.x));
}
double findDistance(struct Point3D &point1, struct Point3D &point2) {
	return sqrt((point1.x - point2.x)*(point1.x - point2.x) +
				(point1.y - point2.y)*(point1.y - point2.y));
}
double VolocityY() {
	return 0;
}
double VolocityX() {
	return 0;
}
double initialVolocity(struct Point3D &point1, struct Point3D &point2) {
	double time = (point2.timestamp - point1.timestamp);
	return findDistance(point1, point2) / time;
}
void rotateAboutYAxis(double q, struct Point3D &point) {
	/*
	z' = z*cos q - x*sin q
	x' = z*sin q + x*cos q
	y' = y
	*/
	point.x = point.z * cos(q) - point.x * sin(q);
	point.z = point.z * sin(q) + point.x * cos(q);
}
struct Point3D PotisionAtPoint(struct Point3D &point) {
	angleXY = angle_XY(initialPoint1, initialPoint2);
	struct Point3D position;
	position.x = point.x;
	double test = cos(angleXY);
	position.y = initialPoint1.y + (point.x * tan(angleXY)) -
		(((point.x * point.x) * GRAVITY)/(2*(initialV * cos(angleXY))*(initialV * cos(angleXY))));
	position.z = point.z;
	cout << position.x << " " << position.y << endl;
	if (position.z != 0.0) {
		angleXZ = angle_XZ(initialPoint1, initialPoint2);
		rotateAboutYAxis(angleXZ, position);
	}
	return position;
}
void calculateObjectPosition(vector<Point3D> &points,struct Point3D point1, struct Point3D point2) {
	initialPoint1 = point1;
	initialPoint2 = point2;
	initialV = initialVolocity(point1, point2);
	struct Point3D temp = point1;
	for (int i = 0;i < MAX_PREDICTION_POINTS;i++) {
		temp.x += 1;
		points.push_back(PotisionAtPoint(temp));
	}
}
//------------------------------------------------------------------------------------------------------------------

int main(int, char**)
{
	//example 3d plot
	vector<Point3D> points;
	struct Point3D point1(1, 50, 1, 1), point2(2, 100, 0, 2);
	calculateObjectPosition(points,point1, point2);
	return 0;
}
