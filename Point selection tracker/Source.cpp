/*Things to try
	Now, previous point data is in oldTrackingPoints[0] ~ [4].
	I need to compare the current tracking point data in trackingPoints[1] with the oldTrackingPoints[4]
	However, there is a bad data which is in negative value. I will need to add if-else that will check whether the data is valid (positve).
	Before begin the checking, add code to check that oldTrackingPoints[4] is not empty.

	Then, make it autonomously add grid point. After running, if grid point is not enough, clear and add new set.
*/

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

bool pointTrackingFlag = false;
bool clearTrackingFlag = false;
Point2f currentPoint;
vector<Point2f> desiredPoint;

// Good Point border criteria default is 160, 480, 240
int borderLeft = 220, borderRight = 420, borderLower = 240, borderHigh = 20;

// Trigger thredshold
int triggerValue = 10;



//detect mouse events
void onMouse(int event, int x, int y, int, void*)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		currentPoint = Point2f((float)x, (float)y);
		pointTrackingFlag = true;
	}

	if (event == CV_EVENT_RBUTTONDOWN)
	{
		clearTrackingFlag = true;
	}
}

int main(int argc, char* argv[])
{
	VideoCapture cap(0);

	// Open VideoCapture from file
	//cap.open("C:/Users/tecnic08/Videos/turn_high_fisheye.avi");

	if (!cap.isOpened())
	{
		cerr << "Unable to open the webcam." << endl;
		return -1;
	}

	// Set the desired point grid
	//int desiredX[6] = {160,224,288,352,416,480};
	//int desiredY[4] = {60,120,180,240};

	// Set the desired point grid FOR FISHEYE
	int desiredX[6] = { 200, 240, 280, 320, 360, 420 };
	int desiredY[6] = { 40,80,120,160,200,240 };
	
		// Push desired (x,y) in vector of desiredPoint
		for (int i = 0; i < 6; i++) 
		{
			for (int j = 0; j < 6; j++)
			{ 
				desiredPoint.push_back(Point2f(desiredX[i], desiredY[j]));
			}
		}

	
		TermCriteria terminationCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.02);

		// Matching box size
		Size windowSize(25, 25);

		// Max number of points
		const int maxNumPoints = 200;

		string windowName = "Tracker";
		namedWindow(windowName, 1);
		setMouseCallback(windowName, onMouse, 0);

		Mat prevGrayImage, curGrayImage, image, frame;
		vector<Point2f> trackingPoints[2];
		vector<Point2f> oldTrackingPoints[5];

		// Image size scaling factor
		float scalingFactor = 1.0;

		//pointTrackingFlag = true;

		while (true)
		{
			cap >> frame;

			if (frame.empty())
				break;

			resize(frame, frame, Size(), scalingFactor, scalingFactor, INTER_AREA);

			frame.copyTo(image);

			cvtColor(image, curGrayImage, COLOR_BGR2GRAY);

			if (!trackingPoints[0].empty())
			{
				vector<uchar> statusVector;
				vector<float> errorVector;

				if (prevGrayImage.empty())
				{
					curGrayImage.copyTo(prevGrayImage);
				}

				calcOpticalFlowPyrLK(prevGrayImage, curGrayImage, trackingPoints[0], trackingPoints[1], statusVector, errorVector, windowSize, 3, terminationCriteria, 0, 0.001);

				int count = 0;
				int minDist = 2;
				int goodPoints = 0;

				for (int i = 0; i < trackingPoints[1].size(); i++)
				{
					
					if (pointTrackingFlag)
					{	// Check if new point are too close.
						if (norm(currentPoint - trackingPoints[1][i]) <= minDist)
						{
							pointTrackingFlag = false;
							continue;
						}
					}

					//cout << "POS of point " << i << "(" << trackingPoints[1][i].x << "," << trackingPoints[1][i].y << ")" << endl;

					// Check if the status vector is good
					if (!statusVector[i])
						continue;

					// Remove tracking point that is out of ROI
					if (trackingPoints[1][i].x < borderLeft || trackingPoints[1][i].x > borderRight)
						continue;
					if (trackingPoints[1][i].y < borderHigh || trackingPoints[1][i].y > borderLower)
						continue;
					
					// Refine the vector to be more efficient
					//trackingPoints[1][count++] = trackingPoints[1][i];

					// Track point icon
					int radius = 8;
					int thickness = 2;
					int lineType = 3;
					circle(image, trackingPoints[1][i], radius, Scalar(0, 255, 0), thickness, lineType);
					goodPoints++;

					// Compare current point with previous point.
					if (!oldTrackingPoints[4].empty()) 
					{
						// Check whether the point that I am comparing is a valid point and not negative.
						if (oldTrackingPoints[4][i].x > 0 && oldTrackingPoints[4][i].y > 0) 
						{
							if (oldTrackingPoints[4][i].y - oldTrackingPoints[3][i].y > 2)
							{
								if (oldTrackingPoints[4][i].y - trackingPoints[1][i].y > triggerValue)
								{
									cout << "Tracking point " << i << " triggered the warning." << endl;
									circle(image, trackingPoints[1][i], radius, Scalar(0, 0, 255), thickness, lineType);
								}
							}
						}
					
					}
				}
				// Number of good tracking point and in ROI
				cout << "Good Points = " << goodPoints << endl;

				// Check whether there is enough good point
				if (goodPoints <= 8)
				{
					clearTrackingFlag = true;
					pointTrackingFlag = true;
					cout << "Tracking grid reset" << endl;
				}

				// Transfer previous point data for movement analysis
				oldTrackingPoints[4] = oldTrackingPoints[3];
				oldTrackingPoints[3] = oldTrackingPoints[2];
				oldTrackingPoints[2] = oldTrackingPoints[1];
				oldTrackingPoints[1] = oldTrackingPoints[0];
				oldTrackingPoints[0] = trackingPoints[1];

				// Resize the vector according to count to be more efficient, but pose a problem to track the previous location. So, this was commented out.
				//trackingPoints[1].resize(count);
			}

			// Clear all current tracking points when right click is detected
			if (clearTrackingFlag)
			{
				//Reset all data
				trackingPoints[1].clear();
				trackingPoints[0].clear();
				oldTrackingPoints[0].clear();
				oldTrackingPoints[1].clear();
				oldTrackingPoints[2].clear();
				oldTrackingPoints[3].clear();
				oldTrackingPoints[4].clear();
				// Reset the clear flag
				clearTrackingFlag = false;
			}

			// Add the desiredPoint if left click is detected
			if (pointTrackingFlag)
			{
				for (int k = 0; k < desiredPoint.size(); k++)
				{
					vector<Point2f> tempPoints;
					tempPoints.push_back(desiredPoint[k]);

					cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1, -1), terminationCriteria);

					trackingPoints[1].push_back(tempPoints[0]);
				}

				pointTrackingFlag = false;
				
			}

			// Refining the location of the feature points
			/*if (pointTrackingFlag && trackingPoints[1].size() < maxNumPoints)
			{
				vector<Point2f> tempPoints;
				tempPoints.push_back(currentPoint);
				
				cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1, -1), terminationCriteria);

				// Check that all point are pushed in
				for (int k = 0; k < desiredPoint.size(); k++)
				{
					cout << "No." << k << "Point (" << desiredPoint[k].x << "," << desiredPoint[k].y << ")" << endl;
				}


				trackingPoints[1].push_back(tempPoints[0]);
				pointTrackingFlag = false;
			}*/

			// Show the output and input video feed
			imshow(windowName, image);
			imshow("Video Input", frame);

			// HighGUI waitkey
			char ch = waitKey(10);

			// ESC Check
			if (ch == 27)
				break;

			// Update 'previous' to 'current' point vector
			std::swap(trackingPoints[1], trackingPoints[0]);

			// Update previous image to current image
			cv::swap(prevGrayImage, curGrayImage);
		}

	return 0;
}

