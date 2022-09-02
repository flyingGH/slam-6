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

#include <boost/format.hpp>
#include "vo_features.h"

using namespace cv;
using namespace std;

const int MAX_FRAME = 1000;
const int MIN_NUM_FEAT = 2000;
const string root_path = "/mnt/d/datasets/KITTI/odometry";

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id) {
    ifstream my_file(root_path + "/data_odometry_poses/dataset/poses/00.txt");
    // read line buffer
    string line;
    // read line number
    int i = 0;
    // current coordinates
    double x = 0, y = 0, z = 0;
    // previous coordinates
    double x_prev, y_prev, z_prev;
    if (my_file.is_open()) {
        while ((getline(my_file, line)) && (i <= frame_id)) {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            // cout << line << '\n';
            for (int j = 0; j < 12; j++) {
                in >> z;
                if (j == 7) y = z;
                if (j == 3) x = z;
            }
            i++;
        }
        my_file.close();
    } else {
        cout << "Unable to open file";
        return 0;
    }
    // 3D Pythagoras theorem
    return sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev));
}


int main(int argc, char **argv) {
    double scale = 1.00;

    Mat img_1, img_2;
    {
        const string filename1(root_path + "/data_odometry_color/dataset/sequences/00/image_2/000000.png");
        const string filename2(root_path + "/data_odometry_color/dataset/sequences/00/image_2/000001.png");

        //read the first two frames from the dataset
        const Mat img_1_c = imread(filename1);
        const Mat img_2_c = imread(filename2);

        if (!img_1_c.data || !img_2_c.data) {
            std::cout << " --(!) Error reading images " << std::endl;
            return -1;
        }

        // we work with grayscale images
        cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
        cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
    }

    // feature detection, tracking
    vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
    featureDetection(img_1, points1);        //detect features in img_1
    vector<uchar> status;
    featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

    // TODO: add a function to load these values directly from KITTI's calib files
    // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
    const double focal = 718.8560;
    const cv::Point2d pp(607.1928, 185.2157);
    //recovering the pose and the essential matrix
    Mat E, R, t, mask;
    E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, R, t, focal, pp, mask);

    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;

    Mat R_f(R.clone());
    Mat t_f(t.clone());

    const clock_t begin = clock();

    namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
    namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
        string filename =
                root_path +
                (boost::format("/data_odometry_color/dataset/sequences/00/image_2/%06d.png") % numFrame).str();
        //cout << numFrame << endl;
        const Mat currImage_c = imread(filename);
        cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);

        // optical flow
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
        // 5-point algorithm
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

        Mat prevPts(2, int(prevFeatures.size()), CV_64F);
        Mat currPts(2, int(currFeatures.size()), CV_64F);

        //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
        for (int i = 0; i < prevFeatures.size(); i++) {
            prevPts.at<double>(0, i) = prevFeatures.at(i).x;
            prevPts.at<double>(1, i) = prevFeatures.at(i).y;

            currPts.at<double>(0, i) = currFeatures.at(i).x;
            currPts.at<double>(1, i) = currFeatures.at(i).y;
        }

        scale = getAbsoluteScale(numFrame);

        //cout << "Scale is " << scale << endl;

        if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

            t_f = t_f + scale * (R_f * t);
            R_f = R * R_f;

        } else {
            //cout << "scale below 0.1, or incorrect translation" << endl;
        }

        // lines for printing results
        // myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

        // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
        if (prevFeatures.size() < MIN_NUM_FEAT) {
            //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
            //cout << "trigerring redection" << endl;
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        {
            const int x = int(t_f.at<double>(0)) + 300;
            const int y = int(t_f.at<double>(2)) + 100;
            circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
        }

        rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), FILLED);

        {
//            char text[100];
//            sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
//                    t_f.at<double>(2));
            string text = (boost::format("Coordinates: x = %02fm y = %02fm z = %02fm")
                           % t_f.at<double>(0) % t_f.at<double>(1) % t_f.at<double>(2)).str();

            const int fontFace = FONT_HERSHEY_PLAIN;
            const double fontScale = 1;
            const int thickness = 1;
            const cv::Point textOrg(10, 50);
            putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        }

        imshow("Road facing camera", currImage_c);
        imshow("Trajectory", traj);

        waitKey(1);

    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;

    //cout << R_f << endl;
    //cout << t_f << endl;

    return 0;
}