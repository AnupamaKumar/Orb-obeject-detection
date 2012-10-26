#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"
//#include "projectHeaders.h"

#define MAX_HEIGHT 500
#define MAX_WIDTH 500

#define MAX_POINTS 20

using namespace std;
using namespace cv;

void featureDetection(Mat img_object, Mat img_scene);
void downsample(Mat *image);
vector<Point2f> nearest(Mat object, Mat image);
void drawCorners(Mat *outputFrame, vector<Point2f> scene_corners, int colour);
void displayImage(String imageName, Mat image, int colour);
void kalman(Mat img, int x, int y);
int getAverage(vector<Point2f> corners, int xOrY) ;
bool shouldDraw(float measure_x, float measure_y, float predict_x,float predict_y);

int main(int argc, char *argv[], char *window_name) {
	if (argc != 5) {
		cout << "Not enough parameters" << endl;
		return -1;
	}

	stringstream conv;


	VideoCapture capture;
	capture.open(atoi(argv[1]));

	const string compareImage1 = argv[2];
	const string compareImage2 = argv[3];
	const string compareImage3 = argv[4];

	Mat image1 = imread(compareImage1, -1);
	Mat image2 = imread(compareImage2, -1);
	Mat image3 = imread(compareImage3, -1);

	downsample(&image1);
	downsample(&image2);
	downsample(&image3);

	displayImage("Image1", image1, 0);
	displayImage("Image2", image2, 1);
	displayImage("Image3", image3, 2);


	//cv::cvtColor(image1, image1, CV_BGR2GRAY);
	//   cv::threshold(image1, image1, 128, 255, CV_THRESH_BINARY);
	//vector<std::vector<cv::Point> > storage;
	//Mat contoursImg1 = image1.clone();
	//findContours(contoursImg1, storage, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	Mat frame;
	Mat grayFrame;
	capture >> frame;
	int frameCounter = 0;

	//KalmanFilter kalman = KalmanFilter(2, 2, 0);

	
    ///kalman.transitionMatrix 
	//	=(Mat_<int>(2,2) << 1, 0, 1, 0);

    //setIdentity(kalman.measurementMatrix);
    //setIdentity(kalman.measurementNoiseCov, Scalar::all(1e-5));
	//setIdentity(kalman.errorCovPost, Scalar::all(1));


KalmanFilter KF(4, 2, 0);
KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

 
// init...

setIdentity(KF.measurementMatrix);
setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
setIdentity(KF.errorCovPost, Scalar::all(.1));

	while (!frame.empty()) {

		Mat prediction = KF.predict();

			//prediction.at<int>(0,0);

            cout << "Prediction: " << prediction << "   ";

		//process only grey frames:
		cvtColor(frame, grayFrame, CV_RGB2GRAY);

		//downsample(&grayFrame);
		//nearest(image1, grayFrame);

		vector<Point2f> image1Corners = nearest(image1, frame.clone());
		vector<Point2f> image2Corners = nearest(image2, frame.clone());
		vector<Point2f> image3Corners = nearest(image3, frame.clone());


		Mat measurement = 
			(Mat_<float>(2,1) << (image1Corners[0].x + image1Corners[2].x)/2, (image1Corners[0].y + image1Corners[2].y)/2); 
		

		cout << measurement << endl;

		if(measurement.at<float>(0,0) != 0){
			KF.correct(measurement);
			Point predictCentre(prediction.at<float>(0,0), prediction.at<float>(1,0));
			cout << predictCentre;
			cv::circle(frame,predictCentre,5,Scalar(255,255,255, 0),3,8,0);
		}


		if(shouldDraw((image1Corners[0].x + image1Corners[2].x)/2, (image1Corners[0].y + image1Corners[2].y)/2, prediction.at<float>(0,0),prediction.at<float>(1,0))){
			drawCorners(&frame, image1Corners, 0);
		}
		drawCorners(&frame, image2Corners, 1);
		drawCorners(&frame, image3Corners, 2);


		cv::circle(frame,cvPoint( (image1Corners[0].x + image1Corners[2].x)/2, (image1Corners[0].y + image1Corners[2].y)/2),5,Scalar(255,255,0, 0),3,8,0);
		


		//cv::circle(frame,cvPoint( getAverage(image1Corners, 0), getAverage(image1Corners,1)),5,Scalar(255,0,255, 0),3,8,0);
		imshow( "Good Matches", frame );

		//post process!
		char key = (char)waitKey(2);
		switch (key) {
		case 27: //what is this for lol
		case 's':
			frameCounter = 0;
			break;
		case 'q':
			return 0;
			break;
		}
		frameCounter++;

		if (frameCounter < 5000) {
			capture >> frame;


		}
	}

	return 0;
}

bool shouldDraw(float measure_x, float measure_y, float predict_x,float predict_y){
	return (((measure_x -  predict_x)*(measure_x -  predict_x) + (measure_y -  predict_y)*(measure_y -  predict_y)) > 50);
}

int getAverage(vector<Point2f> corners, int xOrY) {
	int total = 0;
	for (int i = 0; i < 4; i++) {
		if (xOrY = 0) {
			total = total + corners[i].x;
		} else {
			total = total + corners[i].y;
		}
	}
	return total/4;
}
void drawCorners(Mat *outputFrame, vector<Point2f> scene_corners, int colour) {
	int blue = 0; 
	int green = 0;
	int red = 0;
	int colourCode = colour%3;

	switch(colourCode) {
	case 0:
		blue = 255;
		break;
	case 1:
		green = 255;
		break;
	case 2:
		red = 255;
		break;
	}

	//Draw lines between the corners (the mapped object in the scene image )
	line( *outputFrame, scene_corners[0], scene_corners[1], Scalar(blue, green, red), 4 );
	line( *outputFrame, scene_corners[1], scene_corners[2], Scalar(blue, green, red), 4 );
	line( *outputFrame, scene_corners[2], scene_corners[3], Scalar(blue, green, red), 4 );
	line( *outputFrame, scene_corners[3], scene_corners[0], Scalar(blue, green, red), 4 );

	//cv::circle(*outputFrame,cvPoint( (scene_corners[0].x + scene_corners[2].x)/2, (scene_corners[0].y + scene_corners[2].y)/2),5,Scalar(255,255,0, 0),3,8,0);
}

vector<Point2f> drawBorder(Mat image) {
	std::vector<Point2f> obj_corners(4);

	//Get the corners from the object
	obj_corners[0] = cvPoint(0,0);
	obj_corners[1] = cvPoint( image.cols, 0 );
	obj_corners[2] = cvPoint( image.cols, image.rows );
	obj_corners[3] = cvPoint( 0, image.rows );

	return obj_corners;
}

void displayImage(String imageName, Mat image, int colour) {
	Mat newImage = image.clone();
	drawCorners(&newImage, drawBorder(image), colour);
	imshow(imageName, newImage);
}

void downsample(Mat *image) {
	Mat modifyImage = *image;
	int height = modifyImage.rows;
	int width = modifyImage.cols;

	//account for odds
	if (height%2 != 0) {
		height--;
	}
	if (width%2 != 0) {
		width--;
	}
	//form new images:
	Mat evenSize(modifyImage, Rect(0, 0, width - 1, height - 1));
	Mat downSize;
	while (height * width > 400 * 400) { 
		pyrDown(evenSize, downSize, Size(width/2, height/2));
		//set new image to the downsized one
		*image = downSize;
		printf("Fuck1\n");
		//do again and account for odds
		height = downSize.rows;
		width = downSize.cols;

		if (height%2 != 0) {
			height--;
		}
		if (width%2 != 0) {
			width--;
		}
		printf("Size is now %d by %d\n", width, height);
		Mat next(downSize, Rect(0, 0, width - 1, height - 1));
		evenSize = next;
		printf("Fuck2\n");
	}	
}


void getKeyPoints(Mat object) {
	


}


vector<Point2f> nearest(Mat object, Mat image) {
	//Detect the keypoints using SURF Detector


	//SurfFeatureDetector detector( 300,3,4);
	cv::ORB orb;
	vector<KeyPoint> kp_object;
	vector<KeyPoint> kp_image;

	Mat des_image, des_object, img_matches;

	orb(object,Mat(),kp_object,des_object);
	orb(image,Mat(),kp_image,des_image);

	//detector.detect( object, kp_object );

	//Calculate descriptors (feature vectors)
		//SurfDescriptorExtractor extractor;
		//Mat des_object;

		//extractor.compute( object, kp_object, des_object );

	//FlannBasedMatcher matcher;
	BruteForceMatcher<HammingLUT> matcher;
	namedWindow("Good Matches");

	std::vector<Point2f> obj_corners(4);

	//Get the corners from the object
	obj_corners[0] = cvPoint(0,0);
	obj_corners[1] = cvPoint( object.cols, 0 );
	obj_corners[2] = cvPoint( object.cols, object.rows );
	obj_corners[3] = cvPoint( 0, object.rows );

	char key = 'a';
	int framecount = 0;




	std::vector<vector<DMatch > > matches;
	std::vector<DMatch > good_matches;
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	std::vector<Point2f> scene_corners(4);
	Mat H;


	//detector.detect( image, kp_image );
	//extractor.compute( image, kp_image, des_image );

	matcher.knnMatch(des_object, des_image, matches, 2);

	int maxGoodNumberCounts = 0;

	for(int i = 0; i < min(des_image.rows-1,(int) matches.size()) && maxGoodNumberCounts < MAX_POINTS; i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
	{
		if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
		{
			good_matches.push_back(matches[i][0]);
			maxGoodNumberCounts++;

		}
	}

	//Draw only "good" matches
	//drawMatches( object, kp_object, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if (good_matches.size() >= 4) {
		for( int i = 0; i < good_matches.size(); i++ ) {
			//Get the keypoints from the good matches
			obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
			scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
		}

		H = findHomography( obj, scene, CV_RANSAC );

		perspectiveTransform( obj_corners, scene_corners, H);
	}

	return scene_corners;
}

void findOjbect(Mat object, Mat scene) {


}




void kalman(Mat img, int x, int y){

    KalmanFilter kalman = KalmanFilter(2, 2, 0);
    Mat state = Mat(2, 1, CV_32FC1);  // (phi, delta_phi)

    Mat measurement = Mat(2, 1, CV_32FC1);
    Mat measurement_noise = Mat(2, 1, CV_32FC1);


    float m[2][2] = {{1, 1}, {0, 1}};
    kalman.transitionMatrix = Mat(2, 2, CV_32FC1, m);

    setIdentity(kalman.measurementMatrix, Scalar(1,1));
    setIdentity(kalman.measurementNoiseCov, Scalar(1e-1,1e-1));
    setIdentity(kalman.errorCovPost, Scalar(1));



            // predict a measurement
            Mat prediction = kalman.predict();
            //Point predict_pt = Point(prediction.at<float>(0, 0),prediction.at<float>(0, 1));

			prediction.at<int>(0,0);


            // generate measurement with measurement noise
           // measurement = kalman.measurementMatrix * state + measurement_noise;

            kalman.correct(measurement);


		}


