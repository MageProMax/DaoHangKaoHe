#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/objdetect.hpp>
#include<opencv2/opencv.hpp>

#include<iostream>


#define CMOS_SIZE_L 2240
#define CMOS_SIZE_W 1260
#define TAG 150
#define FOCAL_DISTANCE 1120
using namespace std;
using namespace cv;

typedef struct HSV_maskRange {
	int Hmax = 0;
	int Hmin = 0;
	int Smax = 0;
	int Smin = 0;
	int Vmax = 0;
	int Vmin = 0;
}HMR;

//内参矩阵
Mat camera_matrix = (Mat_<double>(3, 3) << 1120, 0, 1120,
	                                        0, 1120, 630,
	                                         0, 0, 1);


//颜色分离，获取tag的大致轮廓
Mat findcounter(Mat img, HMR img_hmr);

//去除部分噪声，框选tag
vector<Point> getContours(Mat res, Mat img);

//计算tag距离
float Distance(vector<Point> boundRect);


//计算相机坐标系下的坐标及角度
void Camera_coor(Mat img, vector<Point> boundRect);


int main() {
	string path1 = "2025赛季导航组考核资料\\1.png";
	string path2 = "2025赛季导航组考核资料\\2.png";
	Mat img1,img1HSV,img1Gray,img1Gau,img1Canny;
	img1 = imread(path1);
	//resize(img1, img1, Size(), 0.5, 0.5);
	cvtColor(img1, img1HSV, COLOR_BGR2HSV);
	cvtColor(img1, img1Gray, COLOR_BGR2GRAY);

	GaussianBlur(img1HSV, img1Gau, Size(5,5),0,0);
	Canny(img1Gau, img1Canny, 150, 75);
	HMR img1_hmr;
	img1_hmr.Hmax = 179;
	img1_hmr.Hmin = 145;
	img1_hmr.Smax = 250;
	img1_hmr.Smin = 66;
	img1_hmr.Vmax = 135;
	img1_hmr.Vmin = 92;

	Mat res = findcounter(img1HSV, img1_hmr);
	vector<Point> boundRect1=getContours(res, img1);
	float distance1 = Distance(boundRect1)/1000;
	char unit = 'm';
	stringstream s1;
	
	s1 << "distance:"<<distance1;
	string text1 = s1.str();
	text1.push_back(unit);
	Point position = boundRect1[2];
	putText(img1, text1, { position.x + 3,position.y -7}, FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 0),2);
	Camera_coor(img1, boundRect1);
	namedWindow("11", WINDOW_KEEPRATIO);
	imshow("11", img1);




	//////***********调试mask的值*************
	//Mat mask;
	//namedWindow("Trackbar1", (640, 200));
	//createTrackbar("Hmax", "Trackbar1", &img1_hmr.Hmax, 180);
	//createTrackbar("Hmin", "Trackbar1", &img1_hmr.Hmin, 180);
	//createTrackbar("Smax", "Trackbar1", &img1_hmr.Smax, 255);
	//createTrackbar("Smin", "Trackbar1", &img1_hmr.Smin, 255);
	//createTrackbar("Vmax", "Trackbar1", &img1_hmr.Vmax, 255);
	//createTrackbar("Vmin", "Trackbar1", &img1_hmr.Vmin, 255);
	//
 // 
	//namedWindow("img1", WINDOW_AUTOSIZE);
	//namedWindow("img1HSV", WINDOW_AUTOSIZE);
	//namedWindow("img1Gray", WINDOW_AUTOSIZE);
	//namedWindow("img1Canny", WINDOW_AUTOSIZE);
	//namedWindow("img1Gau", WINDOW_AUTOSIZE);

	//while(1) {
	//	Scalar lower(img1_hmr.Hmin, img1_hmr.Smin, img1_hmr.Vmin);
	//	Scalar upper(img1_hmr.Hmax, img1_hmr.Smax, img1_hmr.Vmax);
	//	inRange(img1Gau, lower, upper, mask);
	//	/*imshow("img1", img1);*/
	//	imshow("img1HSV", img1HSV);
	///*	imshow("img1Gray", img1Gray);
	//	imshow("img1Gau", img1Gau);
	//	imshow("img1Canny", img1Canny);*/
	//	imshow("mask", mask);
	//	waitKey(1);
	//};


	waitKey(0);

	

}




vector<Point> getContours(Mat res, Mat img)
{

	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;

	findContours(res, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	//drawContours(img, contours, -1, Scalar(255, 0, 255), 2);
	vector<vector<Point>>conPoly(contours.size());
	
	for (int i = 0; i < contours.size(); i++) {
		int area = contourArea(contours[i]);
		//cout << area << endl;

		if (area < 4500 && area>500) {
			float peri = arcLength(contours[i], true);
			
			approxPolyDP(contours[i], conPoly[i], 0.04 * peri, true);
			drawContours(img, contours, i, Scalar(255, 0, 255), 2);
			Rect Rectcorner = boundingRect(conPoly[i]);
			Point tl = { Rectcorner.tl().x - 5,Rectcorner.tl().y - 5 };
			Point br = { Rectcorner.br().x + 5,Rectcorner.br().y + 5 };
			

			line(img, conPoly[i][0], conPoly[i][2], Scalar(255, 254, 145), 1);
			line(img, conPoly[i][1], conPoly[i][3], Scalar(255, 254, 145), 1);
		
			cout <<conPoly[i] << endl;
			
			circle(img, conPoly[i][0], 5, Scalar(0,0, 255), -1);
			circle(img, conPoly[i][1], 5, Scalar(0, 0, 255), -1);
			circle(img, conPoly[i][2], 5, Scalar(0, 0, 255), -1);
			circle(img, conPoly[i][3], 5, Scalar(0, 0, 255), -1);
			
			rectangle(img, tl,br, Scalar(0, 255, 0), 2);
			return conPoly[i];


		}
	}

}

float Distance(vector<Point> boundRect)
{
	Point a = boundRect[0];
	Point b = boundRect[3];
	float len = norm(a-b);
	cout << "距离" << len;
	if (len - (b.x - a.x) < (b.x - a.x) / 10) {
		float distance = (FOCAL_DISTANCE * 0.265 * TAG) / (len * 0.265);
		return distance;
	}
}


void Camera_coor(Mat img, vector<Point> boundRect)
{
	Point pointbr = boundRect[2];
	Point pointtl = boundRect[0];
	Point pointtr = boundRect[3];
	Point pointbl = boundRect[1];
	
	vector<Point2f>imagePoints/*=vector<Point2f>{ {730,750},{790,747},{730,822},{790,813}}*/;
	vector<Point3f>WorldPoints=vector<Point3f>{{-75,-75,0},{75,-75,0},{-75,75,0},{75,75,0}};
	imagePoints.push_back(pointtl);
	imagePoints.push_back(pointtr);
	imagePoints.push_back(pointbl);
	imagePoints.push_back(pointbr);



	//旋转，平移，畸变
	Mat rvec = Mat::zeros(3, 1, CV_64FC1);
	Mat tvec = Mat::zeros(3, 1, CV_64FC1);
	Mat dist = (Mat_<double>(5,1) <<0, 0, 0, 0, 0);

	int a=solvePnP(WorldPoints, imagePoints, camera_matrix, dist, rvec, tvec, false, SOLVEPNP_EPNP);
	Mat rmatrix;
	Rodrigues(rvec, rmatrix);
	
	//法线
	Mat normal_origin = (Mat_<double>(3, 1) << 0, 0, 1);
	Mat normal_carma = rmatrix * normal_origin + tvec;
	double angle = acos(normal_origin.dot(normal_carma) / (norm(normal_carma) * norm(normal_origin)))*180/CV_PI;
	cout << "角度" << angle;
	stringstream s2;
	s2 << "angle:" << angle;
	string text_angle = s2.str();
	
	Point3f center = {75,75,0};
	Mat center1 = (Mat_<double>(3, 1) << center.x, center.y,center.z);
	Mat center_camMat = rmatrix * center1 + tvec;

	//Point3f center_cam(center_camMat.at<float>(0, 0), center_camMat.at<float>(0, 0), center_camMat.at<float>(0, 0));
	stringstream s1;
	s1 << "pointbr"<<"(" << center_camMat.at<double>(0,0) << ", " << center_camMat.at<double>(1, 0) << ", " << center_camMat.at<double>(2, 0) << ")";
	string text = s1.str();
	putText(img, text, { pointbr.x +2,pointbl.y + 18 }, FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 255),2);
	putText(img, text_angle, { pointbr.x +2,pointbl.y +42 }, FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2);
}





Mat findcounter(Mat imgHSV,HMR img_hmr)
{
	Mat mask, res, res1;
	Scalar lower(img_hmr.Hmin, img_hmr.Smin, img_hmr.Vmin);
	Scalar upper(img_hmr.Hmax, img_hmr.Smax, img_hmr.Vmax);
	

	inRange(imgHSV, lower, upper, mask);

	
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	
	dilate(mask, mask, kernel);
	bitwise_and(imgHSV, imgHSV, res1, mask);
	Canny(res1, res, 75,25);
	
	return res;
}
