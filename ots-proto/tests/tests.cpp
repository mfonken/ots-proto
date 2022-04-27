//
//  tests.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 4/21/22.
//

#include "tests.hpp"

using namespace cv;

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

static void help()
{
    printf( "\nExample of c calls to OpenCV's Kalman filter.\n"
"   Tracking of rotating point.\n"
"   Point moves in a circle and is characterized by a 1D state.\n"
"   state_k+1 = state_k + speed + process_noise N(0, 1e-5)\n"
"   The speed is constant.\n"
"   Both state and measurements vectors are 1D (a point angle),\n"
"   Measurement is the real state + gaussian noise N(0, 1e-1).\n"
"   The real and the measured points are connected with red line segment,\n"
"   the real and the estimated points are connected with yellow line segment,\n"
"   the real and the corrected estimated points are connected with green line segment.\n"
"   (if Kalman filter works correctly,\n"
"    the yellow segment should be shorter than the red one and\n"
"    the green segment should be shorter than the yellow one)."
            "\n"
"   Pressing any key (except ESC) will reset the tracking.\n"
"   Pressing ESC will stop the program.\n"
            );
}

int kalman_test(int, const char**)
{
    help();
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(2, 1, 0);
    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
    Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    char code = (char)-1;
    for(;;)
    {
        img = Scalar::all(0);
        state.at<float>(0) = 0.0f;
        state.at<float>(1) = 2.f * (float)CV_PI / 6;
        KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(KF.errorCovPost, Scalar::all(1));
        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
        for(;;)
        {
            Point2f center(img.cols*0.5f, img.rows*0.5f);
            float R = img.cols/3.f;
            double stateAngle = state.at<float>(0);
            Point statePt = calcPoint(center, R, stateAngle);
            Mat prediction = KF.predict();
            double predictAngle = prediction.at<float>(0);
            Point predictPt = calcPoint(center, R, predictAngle);
            // generate measurement
            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
            measurement += KF.measurementMatrix*state;
            double measAngle = measurement.at<float>(0);
            Point measPt = calcPoint(center, R, measAngle);
            // correct the state estimates based on measurements
            // updates statePost & errorCovPost
            KF.correct(measurement);
            double improvedAngle = KF.statePost.at<float>(0);
            Point improvedPt = calcPoint(center, R, improvedAngle);
            // plot points
            img = img * 0.2;
            drawMarker(img, measPt, Scalar(0, 0, 255), cv::MARKER_SQUARE, 5, 2);
            drawMarker(img, predictPt, Scalar(0, 255, 255), cv::MARKER_SQUARE, 5, 2);
            drawMarker(img, improvedPt, Scalar(0, 255, 0), cv::MARKER_SQUARE, 5, 2);
            drawMarker(img, statePt, Scalar(255, 255, 255), cv::MARKER_STAR, 10, 1);
            // forecast one step
            Mat test = Mat(KF.transitionMatrix*KF.statePost);
            drawMarker(img, calcPoint(center, R, Mat(KF.transitionMatrix*KF.statePost).at<float>(0)),
                       Scalar(255, 255, 0), cv::MARKER_SQUARE, 12, 1);
            line( img, statePt, measPt, Scalar(0,0,255), 1, LINE_AA, 0 );
            line( img, statePt, predictPt, Scalar(0,255,255), 1, LINE_AA, 0 );
            line( img, statePt, improvedPt, Scalar(0,255,0), 1, LINE_AA, 0 );
            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
            state = KF.transitionMatrix*state + processNoise;
            imshow( "Kalman", img );
            code = (char)waitKey(1000);
            if( code > 0 )
                break;
        }
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }
    return 0;
}

void kalman2d_test()
{
    kalman2d_t kf;
    Kalman2D.init( &kf, 1e-5, 0.1, 0.1 );
    help();
    Mat img(500, 500, CV_8UC3);
    Point2f center(img.cols*0.5f, img.rows*0.5f);
    float R = img.cols/3.f;
    char code = (char)-1;
    
    for(;;)
    {
        img = Scalar::all(0);
        double angle = 0;
        double angle_step = 10;
        int markerSize = 10;
        for(;;)
        {
            Kalman2D.predict( &kf );
            Point predictPt = Point(kf.state.px, kf.state.py);
            
            int ang_ = (int)angle % 90;
            float R_ = R / (ang_ < 45 ? cos(ang_ / 180.0 * CV_PI) : sin(ang_ / 180.0 * CV_PI));
            
            Point statePt = calcPoint(center, R_, angle / 180.0 * CV_PI);
            Point measPt = Point(statePt.x, statePt.y);
            double state[2] = { (double)statePt.x, (double)statePt.y };
            Kalman2D.update( &kf, state );
            Point improvedPt = Point(kf.state.px, kf.state.py);
            
            img = img * 0.2;
            drawMarker(img, predictPt, Scalar(0, 255, 255), cv::MARKER_SQUARE, markerSize, 2);
            drawMarker(img, measPt, Scalar(0, 0, 255), cv::MARKER_DIAMOND, markerSize, 2);
            drawMarker(img, improvedPt, Scalar(0, 255, 0), cv::MARKER_TRIANGLE_UP, markerSize, 2);
            drawMarker(img, statePt, Scalar(255, 255, 255), cv::MARKER_STAR, markerSize*2, 1);
//            // forecast one step
//            Mat test = Mat(KF.transitionMatrix*KF.statePost);
//            drawMarker(img, calcPoint(center, R, Mat(KF.transitionMatrix*KF.statePost).at<float>(0)),
//                       Scalar(255, 255, 0), cv::MARKER_SQUARE, 12, 1);
            line( img, statePt, measPt, Scalar(0,0,255), 1, LINE_AA, 0 );
            line( img, statePt, predictPt, Scalar(0,255,255), 1, LINE_AA, 0 );
            line( img, statePt, improvedPt, Scalar(0,255,0), 1, LINE_AA, 0 );
//            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//            state = KF.transitionMatrix*state + processNoise;
            imshow( "Kalman", img );
            code = (char)waitKey(75);
            if( code > 0 )
                break;
            
            angle += angle_step;
            if(angle >= 360)
                angle = 0;
        }
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }
}
