/* from https://github.com/avisingh599/mono-vo */
/*
The MIT License
Copyright (c) 2015 Avi Singh
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <cctype>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

/**
 * calc optical flow and remove outliers
 * @param img_1 previous image
 * @param img_2 current image
 * @param points1 previous feature points
 * @param points2 current feature points
 * @param status valid statuses for optical flows
 */
void featureTracking(const Mat &img_1, const Mat &img_2, vector<Point2f> &points1, vector<Point2f> &points2,
                     vector<uchar> &status) {
    //this function automatically gets rid of points for which tracking fails
    vector<float> err;
    Size winSize = Size(21, 21);
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2,
                         status, err, winSize, 3, criteria, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++) {
        Point2f pt = points2.at(i - indexCorrection);
        // invalid optical flow or outside the frame
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                // define outside points also as invalid
                status.at(i) = 0;
            }
            // remove from feature points list
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            // shift index because of remove points
            indexCorrection++;
        }

    }

}

/**
 * detect feature points using FAST algorithm
 * @param img_1 target image (input)
 * @param points1 feature points (output)
 */
void featureDetection(const Mat &img_1, vector<Point2f> &points1) {
    //uses FAST as of now, modify parameters as necessary
    int fast_threshold = 20;
    bool non_max_suppression = true;
    vector<KeyPoint> key_points_1;
    FAST(img_1, key_points_1, fast_threshold, non_max_suppression);

    KeyPoint::convert(key_points_1, points1, vector<int>());
}