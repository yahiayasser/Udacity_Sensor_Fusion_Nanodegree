/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

#include "Project_CFG.h"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // store calibration data in OpenCV matrices
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector

    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img;
        img = cv::imread(imgFullFilename);

        DataFrame frame;
        frame.cameraImg = img;

        // push image into data frame buffer
        if (dataBuffer.size() == dataBufferSize) {
             dataBuffer.erase(std::begin(dataBuffer)); 
        }

        dataBuffer.push_back(frame);

        /* Object Detection */
        float confThreshold = 0.2;
        float nmsThreshold = 0.4;
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold,
            nmsThreshold, yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, false);

        cout << "Detect objects done" << endl;

        /* Crop lidar points */
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        /* remove lidar points based on distance properties */
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1;
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;
        cout << "Crop lidar points done" << endl;

        /* Cluster lidar points cloud */
        float shrinkFactor = 0.1;
        clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor,
            P_rect_00, R_rect_00, RT);

        //visualize 3D objects
        bVis = false;
        if(bVis)
        {
            show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            bVis = false;
        }
        cout << "Cluster lidar points done" << endl;


        //Comment to continue to the next part
        //continue;


        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);
        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = DETECTOR_TYPE;

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, DETECTOR_TYPE, false);
        }
        cout << "Detect keypoints done" << endl;

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = false;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // ...
            std::vector<cv::KeyPoint> CarPoints;
            int vehicleKeyPointsNumber = 0;
            for (auto point : keypoints) {
               if (vehicleRect.contains(cv::Point2f(point.pt))) { 
                   CarPoints.push_back(point); 
                   vehicleKeyPointsNumber++;
                }
            }
            keypoints = CarPoints;  
            cout << "Number of key points on the vehicle is " + to_string(vehicleKeyPointsNumber) << endl;   
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 20;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
        cv::Mat descriptors;
        string descriptorType = DESCRIPTOR_TYPE; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = MATCHER_TYPE;        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = SELECTOR_TYPE;       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            cout << "Matched " << matches.size() << " keypoints" << endl;

            /* Track 3D object Bounding Boxes */
            map<int, int> bbBestMatches;
            /* Match the list of 3D objects (vactor<BoundingBox>) between current and previous frame */

            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2), *(dataBuffer.end() - 1));
            for(int it = 0; it < bbBestMatches.size(); it++)
            {
                std::cout<< to_string(it) +"  "+ to_string(bbBestMatches[it])<< std::endl;
            }
            (dataBuffer.end()-1) -> bbMatches = bbBestMatches;

            /* Compute TTC in object in front */
            for(auto it1 = (dataBuffer.end()-1) -> bbMatches.begin(); it1 != (dataBuffer.end()-1) -> bbMatches.end(); it1++)
            {
                BoundingBox *prevBB, *curBB;
                for(auto it2 = (dataBuffer.end()-1) -> boundingBoxes.begin(); it2 != (dataBuffer.end()-1) -> boundingBoxes.end(); it2++)
                {
                    if(it1->second == it2->boxID)
                        curBB = &(*it2);
                }

                for(auto it2 = (dataBuffer.end()-2) -> boundingBoxes.begin(); it2 != (dataBuffer.end()-2) -> boundingBoxes.end(); it2++)
                {
                    if(it1->first == it2->boxID)
                        prevBB = &(*it2);
                }

                if(curBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0)
                {
                    /* Compute TTC based on lidar points */
                    double ttcLidar;

                    computeTTCLidar(prevBB->lidarPoints, curBB->lidarPoints, sensorFrameRate, ttcLidar);
                    cout<<"TTC lidar = " + std::to_string(ttcLidar)<<endl;

                    /* Assign enclosed keypoint matches to bounding boxes */
                    clusterKptMatchesWithROI(*curBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);

                    /* Compute TTC based on camera */
                    double ttcCamera;
                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, curBB->kptMatches, sensorFrameRate, ttcCamera);              

                    cout<<"TTC Camera = " + std::to_string(ttcCamera)<<endl;
                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat visImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        showLidarImgOverlay(visImg, curBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);


                        cv::rectangle(visImg, cv::Point(curBB->roi.x, curBB->roi.y), cv::Point(curBB->roi.x + curBB->roi.width, curBB->roi.y + curBB->roi.height), cv::Scalar(0, 255, 0), 2);

                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg,str,cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, visImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, visImg);
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
            }
        }

    } // eof loop over all images

    return 0;
}
