//
// Created by ajay on 29/1/19.
//

#include <opencv/cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>


using namespace std;
using namespace cv;


int PoseEstimation();
int Calibration();

float CtoW[4][4];

int main( int argc, char** argv) {

    int m;
    cout
            << "\nPlease select the mode to continue: \n \n"
               " Press 1 for Pose Estimation Mode \n\n"
               " Press 2 for Calibration Mode \n\n"
               " Press 0 to Kill the Program \n";

    cin >> m;

    if (m == 1)
        PoseEstimation();
    if (m == 2)
        Calibration();
    if (m == 0)
        return 0;

    /*while(1)
    {
        char key = (char)waitKey(5);
        if (key == 13)
            PoseEstimation();
        if (key == ' ')
            Calibration();
        if (key == 27)
            break;
    }
    return 0;*/
}



int PoseEstimation()
{
// Define the 'capture' object for camera
    VideoCapture capture = VideoCapture("rtsp://ajay:ajay@192.168.1.108/cam/realmonitor?channel=1&subtype=0");  // start camera using the link specified
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frameCount = 15;    //capture.get(CV_CAP_PROP_FPS);

    cout << "Width: " << frameWidth << "Height: " << frameHeight << "FPS: " << frameCount;
    VideoWriter video("PoseEstimation_Trial_720P_FPS_15_Trial_5.avi", CV_FOURCC('M','J','P','G'), frameCount, Size(frameWidth, frameHeight));

    float PtoC[4][4];
    /*float CtoW[4][4] = {0.0942752, 	 0.235729, 	 -0.967235, 	 -0.174507,
                                    0.99479, 	 -0.0601619, 	 0.0822986, 	 -0.148265,
                                    -0.0387905, 	 -0.969955, 	 -0.240173, 	 0.354101,
                                    0, 0, 0, 1};*/

    float PtoW[4][4] = {0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0};

    // Initialize Mat objects for storing image, camera matrix and distortion coffecients
    Mat image, imageCopy;
    Mat cameraMatrix, distCoeffs;
    Mat R = Mat::zeros(3, 3, CV_64F);
    Mat rotVec = Mat::zeros(1, 3, CV_64F);

    // Define dictionary to be used for AruCo tags
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    // Initialize vectors to store corners and IDs of detected aruco tags
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<Vec3d> rvecs, tvecs;

    vector<Vec3f> rotationAngles;
    vector<Vec3f> translationMatrix;

    // Read the camera parameters for D1 Resolution from the specified file
    /*FileStorage fs("Camera_Param_3.xml", FileStorage::READ);
    fs["Intrinsic_Parameters"] >> cameraMatrix;
    fs["Distorion_Coeffecients"] >> distCoeffs;*/

    // Read the camera parameters for 720P Resolution from the specified file
    FileStorage fs("Camera_Param_720P.xml", FileStorage::READ);
    fs["Intrinsic_Parameters"] >> cameraMatrix;
    fs["Distorion_Coeffecients"] >> distCoeffs;

    while (capture.grab()) {
        capture >> image;  // Capture the frame from camera
        image.copyTo(imageCopy);
        aruco::detectMarkers(image, dictionary, corners, ids);  // Function to detect aruco markers

        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < ids.size(); i++)
                aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);


//          #####################             Pose Estimation               ###################
            // Compute Point to Camera RnT Matrix --> (PtoC)
            rotVec.at<float>(0,1) = rvecs[0][1];
            rotVec.at<float>(0,2) = rvecs[0][2];
            rotVec.at<float>(0,3) = rvecs[0][3];
            Rodrigues(rotVec, R);
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    PtoC[i][j] = R.at<double>(i, j);
                    //cout << R.at<double>(i,j) << "\t \t";
                    //if (j == 3) cout << endl;
                    if (i == j && i == 3) {
                        PtoC[0][3] = tvecs[0][0];
                        PtoC[1][3] = tvecs[0][1];
                        PtoC[2][3] = tvecs[0][2];
                        PtoC[3][3] = 1;
                    }
                }
            }

            // Compute 3D Pose of the Aruco Tag
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) {
                        PtoW[i][j] += CtoW[i][k] * PtoC[k][j];
                    }
                }
            }

            // Print 3D Translation vectors of Aruco wrt to World origin
            /*cout << "Translation Vectors of Point in World are: \n";
            for (int i = 0; i < 3; i++)
            {
                cout << PtoW[i][3] << "\t \t";
            }
            cout << endl;*/

        }

        // Store the coordinates of Pw and Pc in string form to print on the image
        char xw[10], yw[10], zw[10], xc[10], yc[10], zc[10];
        sprintf(xw, "X: %f m", PtoW[0][3]);
        sprintf(yw, "Y: %f m", PtoW[1][3]);
        sprintf(zw, "Z: %f m", PtoW[2][3]);

        sprintf(xc, "X: %f m", PtoC[0][3]);
        sprintf(yc, "Y: %f m", PtoC[1][3]);
        sprintf(zc, "Z: %f m", PtoC[2][3]);

        // Print the Pw and Pc coordinates on the image
        putText(imageCopy, "World Coordinates of Robot", Point(7,20), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,0));
        putText(imageCopy, xw, Point(7,40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));
        putText(imageCopy, yw, Point(7,60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));
        putText(imageCopy, zw, Point(7,80), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));

        putText(imageCopy, "Camera Coordinates of Robot", Point(7,100), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,255,255));
        putText(imageCopy, xc, Point(7,120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));
        putText(imageCopy, yc, Point(7,140), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));
        putText(imageCopy, zc, Point(7,160), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,0,255));

        // Display the image
        namedWindow("3D Pose Estimation Window", WINDOW_FREERATIO);
        imshow("3D Pose Estimation Window", imageCopy);

        // Store Video Stream
        video.write(imageCopy);

        //rvecs.empty();
        //tvecs.empty();
        R.empty();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                PtoW[i][j] = 0;
            }
        }

        char key = (char) waitKey(5);
        if (key == 27)
        {
            // Release the camera
            capture.release();
            video.release();
            //cvDestroyWindow("3D Pose Estimation Window");
            destroyAllWindows();
            break;
        }
    }

    cout
            << "Please select the mode to continue: \n \n"
               " Press 1 to go back to Main Program \n\n"
               " Press 2 for Calibration Mode \n\n"
               " Press 0 to Kill the Program \n";
    int m;
    cin >> m;

    if (m == 1)
        return main(0,0);
    if (m == 2)
        Calibration();
    if (m == 0)
        return 0;
}

// ###################################         Calibration Mode           #####################################################################
int Calibration()
{
    float PtoC[4][4], invR[4][4], invT[4][4], inv[4][4];    // CtoW[4][4];

    float PtoW[4][4] = { 1,  0,  0,     0.5,
                         0,  1,  0,     0.2,
                         0,  0,  1,      0,
                         0,  0,  0,      1};

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            invR[i][j] = 0;
            invT[i][j] = 0;
            inv[i][j] = 0;
            CtoW[i][j] = 0;
            if (i == j) invT[i][j] = 1;
        }
    }
    invR[3][3] = 1;

    // Initialize Mat objects for storing image, camera matrix and distortion coffecients
    Mat image, imageCopy;
    Mat cameraMatrix, distCoeffs;
    Mat R = Mat::zeros(3,3,CV_64F);

    // Define dictionary to be used for AruCo tags
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    // Initialize vectors to store corners and IDs of detected aruco tags
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<Vec3d> rvecs, tvecs;

    vector<Vec3f> rotationAngles;
    vector<Vec3f> translationMatrix;

    // Read the camera parameters from the specified file
    FileStorage fs("Camera_Param_720P.xml",FileStorage::READ);
    fs["Intrinsic_Parameters"] >> cameraMatrix;
    fs["Distorion_Coeffecients"] >> distCoeffs;

    // Define the vid object for camera
    VideoCapture capture = VideoCapture("rtsp://ajay:ajay@192.168.1.108/cam/realmonitor?channel=1&subtype=0");  // start camera using the link

    while (capture.grab())
    {
        capture >> image;  // Capture the frame from camera
        image.copyTo(imageCopy);
        aruco::detectMarkers(image, dictionary, corners, ids);  // Function to detect aruco markers

        if (ids.size() > 0)
        {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < ids.size(); i++)
                aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        // Display the image
        namedWindow("out", WINDOW_FREERATIO);
        imshow("out", imageCopy);

        char key = (char)waitKey(5);
        if (key == 27)
            break;
    }

    cout << endl << "Rotation Vetor: " << endl;
    for (const auto i : rvecs)
    {
        cout << i << ' ' ;
    }

    cout << endl << "Translation Vector: " << endl;
    for (const auto i : tvecs)
        cout << i << ' ' ;

    cout << "\n Rodrigues Matrix: \n";
    Rodrigues(rvecs, R);
    for (int i = 0; i < 4; i++)
    {
        for (int j =0; j<4; j++)
        {
            PtoC[i][j] = R.at<double>(i,j);
            cout << R.at<double>(i,j) << "\t \t";
            if (j ==3) cout << endl;
            if (i == j && i == 3)
            {
                PtoC[0][3] = tvecs[0][0];
                PtoC[1][3] = tvecs[0][1];
                PtoC[2][3] = tvecs[0][2];
                PtoC[3][3] = 1;
            }
        }
    }
    cout << endl;

    // Print Rotation Matrix
    cout << "Point to Camera RnT Matrix: \n";
    for (int i = 0; i < 4; i++)
    {
        for (int j =0; j<4; j++)
        {
            cout << PtoC[i][j] << "\t \t";
            if (j ==3) cout << endl;
        }
    }

    // Computing Inverse
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i > 2) invR[i][j] = 0;
            else invR[i][j] = PtoC[j][i];
            if (i == j && i == 3) invR[i][j] = 1;
        }
    }

    invT[0][3] = -tvecs[0][0];
    invT[1][3] = -tvecs[0][1];
    invT[2][3] = -tvecs[0][2];

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                inv[i][j] += invR[i][k] *  invT[k][j];
            }
        }
    }

    cout << endl << "inv(PtoC): " << endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cout << inv[i][j] << " \t \t";
            if (j == 3) cout << endl;
        }
    }

    // Computing CtoW
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                CtoW[i][j] += PtoW[i][k] *  inv[k][j];
            }
        }
    }

    // Print CtoW matrix
    cout << "CtoW: \n";
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cout << CtoW[i][j] << " \t ";
            if (j == 3) cout << endl;
        }
    }

    // Release the camera
    capture.release();
    cvDestroyWindow("out");

    cout
            << "Please select the mode to continue: \n \n"
               " Press 1 for Pose Estimation Mode \n\n"
               " Press 2 to go back to Main Program \n\n"
               " Press 0 to Kill the Program \n";
    int m;
    cin >> m;

    if (m == 1)
        PoseEstimation();
    if (m == 2)
        return main(0,0);
    if (m == 0)
        return 0;
}