
#include <unordered_set>
#include <numeric>
#include "matching2D.hpp"

#include "Project_CFG.h"

using namespace std;

inline bool PointInBoundingBox(DataFrame &frame, int index, int &matchingBoxId) {
	matchingBoxId = -1;
    bool returnVal = false;
    int matchCount = 0;

    const cv::Point2f point(frame.keypoints.at(index).pt);

    for (int BoxID = 0; BoxID < frame.boundingBoxes.size(); BoxID++) {

        if (!(frame.boundingBoxes.at(BoxID).roi.contains(point)))
             continue;

        matchCount++;
        if (matchCount > 1) 
        {
            matchingBoxId = -1;
            return returnVal;
        }

        matchingBoxId = BoxID;
    }

    if(matchingBoxId == -1)
        return returnVal;

    returnVal = true;
	return returnVal;
}

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    { 
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                it2->lidarPoints.push_back(*it1);
                lidarPoints.erase(it1);
                it1--;
                break;
            }
        } // eof loop over all bounding boxes

    } // eof loop over all Lidar points
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, const cv::Size worldSize, const cv::Size imageSize, const bool bWait) {

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin() ; it1 != boundingBoxes.end(); it1++) {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor{ cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150)) };

        // plot Lidar points into top view image
        int top = 1e8, bottom = 0.0, left = 1e8, right = 0.0;
        float xwmin = 1e8, ywmax = 1e8, ywmin = 1e8;

        for (auto it2 = it1 -> lidarPoints.begin() ; it2 != it1 -> lidarPoints.end(); it2++) {
            // world coordinates
            float xw = (*it2).x;         // world position in m with x facing forward from sensor
            float yw= (*it2).y;          // world position in m with y facing left from sensor

            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle

            if (top > y) { top = y; }
            if (bottom > y) { bottom = y; }
            if (left > x) { left = x; }
            if (right > x) { right = x; }

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];

        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)(it1->lidarPoints.size()));
        cv::putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        cv::putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    const int nMarkers = floor(worldSize.height / lineSpacing);

    for (size_t index=  0; index < nMarkers; ++index) {
        int y = (-(index * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";

    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg=nullptr)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    //double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    double medianDisRatio;
    int SIZE = distRatios.size();
    sort(distRatios.begin(), distRatios.end());
    if(SIZE%2 == 1)
    {
        medianDisRatio = distRatios[(SIZE-1)/2];
    }
    else
    {
        medianDisRatio = (distRatios[(SIZE-1)/2] + distRatios[(SIZE)/2]) / 2;
    }

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDisRatio);
}

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double dist_mean = 0;
    std::vector<cv::DMatch>  kptMatches_roi;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint kp = kptsCurr.at(it->trainIdx);
        if (boundingBox.roi.contains(cv::Point(kp.pt.x, kp.pt.y)))
            kptMatches_roi.push_back(*it);
     }
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
         dist_mean += it->distance;
    if (kptMatches_roi.size() > 0)
         dist_mean = dist_mean/kptMatches_roi.size();
    else return;
    double threshold = dist_mean * 0.7;
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
    {
       if (it->distance < threshold)
           boundingBox.kptMatches.push_back(*it);
    }
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    std::vector<double> X_PointsPrev;
    std::vector<double> X_PointsCurr;
    double X_Prev = 1e8;
    double X_Curr = 1e8;
    double dT = 1 / frameRate;

    for(int it1 = 0; it1 < lidarPointsPrev.size(); it1++)
    {
        X_PointsPrev.push_back(lidarPointsPrev[it1].x);
    }

    for(int it2 = 0; it2 < lidarPointsCurr.size(); it2++)
    {
        X_PointsCurr.push_back(lidarPointsCurr[it2].x);
    }

    sort(X_PointsPrev.begin(), X_PointsPrev.end()); 
    sort(X_PointsCurr.begin(), X_PointsCurr.end()); 

    if(X_PointsPrev.size()%2 == 1)
    {
        X_Prev = X_PointsPrev[(X_PointsPrev.size()-1)/2];
    }
    else
    {
        X_Prev = X_PointsPrev[(X_PointsPrev.size())/2];
    }

    if(X_PointsCurr.size()%2 == 1)
    {
        X_Curr = X_PointsCurr[(X_PointsCurr.size()-1)/2];
    }
    else
    {
        X_Curr = (X_PointsCurr[(X_PointsCurr.size())/2] + X_PointsCurr[((X_PointsCurr.size())/2) - 1])/2;
    }

    TTC = (X_Curr * dT) / (X_Prev - X_Curr);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
	int prevFrameBoxesCount = prevFrame.boundingBoxes.size();
	int currFrameBoxesCount = currFrame.boundingBoxes.size();

    std::vector<std::vector<int>> listOfMatches(prevFrameBoxesCount, std::vector<int>(currFrameBoxesCount, 0));

    int prevFrameBoxesIdx, currFrameBoxesIdx;

	for(auto match : matches){
		if(!PointInBoundingBox(prevFrame, match.queryIdx, prevFrameBoxesIdx)) { continue; }
		if(!PointInBoundingBox(currFrame, match.trainIdx, currFrameBoxesIdx)) { continue; }

        ++listOfMatches.at(prevFrameBoxesIdx).at(currFrameBoxesIdx);
	}

    for (int idx1 = 0; idx1 < prevFrameBoxesCount; idx1++) {
        int idx2 = distance(begin(listOfMatches.at(idx1)), max_element(begin(listOfMatches.at(idx1)), end(listOfMatches.at(idx1))));
        if (listOfMatches.at(idx1).at(idx2) != 0)
            bbBestMatches[prevFrame.boundingBoxes.at(idx1).boxID] = currFrame.boundingBoxes.at(idx2).boxID;
    }
}
