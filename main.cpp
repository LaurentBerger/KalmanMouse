#include <iostream>
#include <vector>

//#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/highgui/highgui_c.h>

using namespace cv;
using namespace std;


struct mouse_info_struct { int x,y; };
struct mouse_info_struct mouse_info = {-1,-1}, last_mouse;


            // plot points
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ),                \
Point( center.x + d, center.y + d ), color, 2, LINE_AA, 0); \
line( img, Point( center.x + d, center.y - d ),                \
Point( center.x - d, center.y + d ), color, 2, LINE_AA, 0 )


vector<Point> mousev,kalmanv;

void on_mouse(int event, int x, int y, int flags, void* param) {
 //if (event == CV_EVENT_LBUTTONUP)
 {
  last_mouse = mouse_info;
  mouse_info.x = x;
  mouse_info.y = y;
  
  cout << "got mouse " << x <<","<< y <<endl;
 }
}


int main (int argc, char * const argv[]) 
{
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(6, 2, 0);
    Mat_<float> state(6, 1); 
    Mat processNoise(6, 1, CV_32F);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
    char code = (char)-1;
 
    namedWindow("mouse kalman");
    setMouseCallback("mouse kalman", on_mouse, 0);
 
    for(;;)
    {
        if (mouse_info.x < 0 || mouse_info.y < 0) 
        {
            imshow("mouse kalman", img);
            waitKey(30);
            continue;
        }
        else
            break;
    }
    KF.statePre.at<float>(0) = mouse_info.x;
    KF.statePre.at<float>(1) = mouse_info.y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.statePre.at<float>(4) = 0;
    KF.statePre.at<float>(5) = 0;
    //KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

    KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,1,0,0.5,0, 0,1,0,1,0,0.5, 0,0,1,0,1,0, 0,0,0,1,0,1, 0,0,0,0,1,0, 0,0,0,0,0,1);
    KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,0,0,0.,0, 0,1,0,0,0,0.0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0);
    KF.measurementMatrix = (Mat_<float>(2, 6) << 1,0,1,0,0.5,0, 0,1,0,1,0,0.5);
  
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(5e-1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e1));
    setIdentity(KF.errorCovPost, Scalar::all(.5));

    mousev.clear();
    kalmanv.clear();
    while (1==1)
    {
   
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
   
        measurement(0) = mouse_info.x;
        measurement(1) = mouse_info.y;
        Point measPt(measurement(0),measurement(1));
        cout << norm(predictPt - measPt) << "\t";
        {
            Point2f predictPt(prediction.at<float>(0),prediction.at<float>(1));
            Point2f measPt(measurement(0),measurement(1));
            cout << norm(predictPt - measPt) << "\n";
        }
        mousev.push_back(measPt);

        Mat estimated = KF.correct(measurement);
        Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        kalmanv.push_back(statePt);
   

        img = Scalar::all(0);
        drawCross( statePt, Scalar(255,255,255), 5 );
        drawCross( measPt, Scalar(0,0,255), 5 );
   
        for (int i = 0; i < mousev.size()-1; i++) 
        {
            line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
        }
        for (int i = 0; i < kalmanv.size()-1; i++) 
        {
            line(img, kalmanv[i], kalmanv[i+1], Scalar(0,255,0), 1);
        }
   
   
   
        imshow( "mouse kalman", img );
        code = (char)waitKey(10);
   
        if( code > 0 )
            break;
        }
 
    return 0;
} 



int main2 (int argc, char * const argv[]) {
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(6, 2, 0);
    Mat_<float> state(6, 1); /* (x, y, Vx, Vy) */
    Mat processNoise(6, 1, CV_32F);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
    char code = (char)-1;
 
 namedWindow("mouse kalman");
 setMouseCallback("mouse kalman", on_mouse, 0);
 
    for(;;)
    {
      if (mouse_info.x < 0 || mouse_info.y < 0) 
      {
        imshow("mouse kalman", img);
       waitKey(30);
       continue;
        }
    }
        KF.statePre.at<float>(0) = mouse_info.x;
  KF.statePre.at<float>(1) = mouse_info.y;
  KF.statePre.at<float>(2) = 0;
  KF.statePre.at<float>(3) = 0;
  KF.statePre.at<float>(4) = 0;
  KF.statePre.at<float>(5) = 0;
  //KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

  KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,1,0,0.5,0, 0,1,0,1,0,0.5, 0,0,1,0,1,0, 0,0,0,1,0,1, 0,0,0,0,1,0, 0,0,0,0,0,1);
  KF.measurementMatrix = (Mat_<float>(2, 6) << 1,0,1,0,0.5,0, 0,1,0,1,0,0.5);
  
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
    if (0 == 1)
    {
        cout << KF.transitionMatrix << "\n";
        cout << "\n";
         cout << KF.measurementMatrix << "\n";
        cout << "\n";
       cout << KF.processNoiseCov << "\n";
        cout << "\n";
        cout << KF.measurementNoiseCov << "\n";
        cout << "\n";
        cout << KF.errorCovPre<< "\n";
        cout << "\n";
        cout << KF.errorCovPost << "\n";
        cout << "\n";
        cout << KF.statePre << "\n";
        cout << "\n";
        cout << KF.statePost << "\n";
        cout << "\n";
        cout << KF.gain << "\n";
        cout << "\n";
    }

  mousev.clear();
  kalmanv.clear();
        //randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
  
   while (1==1)
        {
//            Point statePt(state(0),state(1));
   
            Mat prediction = KF.predict();
                    cout << KF.measurementMatrix << "\n";
            if (0 == 1)
            {
                cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n";
                cout << KF.transitionMatrix << "\n";
                cout << "\n";
                    cout << KF.measurementMatrix << "\n";
                cout << "\n";
                cout << KF.processNoiseCov << "\n";
                cout << "\n";
                cout << KF.measurementNoiseCov << "\n";
                cout << "\n";
                cout << KF.errorCovPre<< "\n";
                cout << "\n";
                cout << KF.errorCovPost << "\n";
                cout << "\n";
                cout << KF.statePre << "\n";
                cout << "\n";
                cout << KF.statePost << "\n";
                cout << "\n";
                cout << KF.gain << "\n";
                cout << "\n";
            }
            Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
   
            measurement(0) = mouse_info.x;
   measurement(1) = mouse_info.y;
   
   Point measPt(measurement(0),measurement(1));
   mousev.push_back(measPt);
            // generate measurement
            //measurement += KF.measurementMatrix*state;

   Mat estimated = KF.correct(measurement);
    if (0 == 1)
    {
            cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n";
        cout << KF.transitionMatrix << "\n";
        cout << "\n";
         cout << KF.measurementMatrix << "\n";
        cout << "\n";
       cout << KF.processNoiseCov << "\n";
        cout << "\n";
        cout << KF.measurementNoiseCov << "\n";
        cout << "\n";
        cout << KF.errorCovPre<< "\n";
        cout << "\n";
        cout << KF.errorCovPost << "\n";
        cout << "\n";
        cout << KF.statePre << "\n";
        cout << "\n";
        cout << KF.statePost << "\n";
        cout << "\n";
        cout << KF.gain << "\n";
        cout << "\n";
    }
   Point statePt(estimated.at<float>(0),estimated.at<float>(1));
   kalmanv.push_back(statePt);
   

            img = Scalar::all(0);
            drawCross( statePt, Scalar(255,255,255), 5 );
            drawCross( measPt, Scalar(0,0,255), 5 );
//            drawCross( predictPt, Scalar(0,255,0), 3 );
//   line( img, statePt, measPt, Scalar(0,0,255), 3, CV_AA, 0 );
//   line( img, statePt, predictPt, Scalar(0,255,255), 3, CV_AA, 0 );
   
   for (int i = 0; i < mousev.size()-1; i++) {
    line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
   }
   for (int i = 0; i < kalmanv.size()-1; i++) {
    line(img, kalmanv[i], kalmanv[i+1], Scalar(0,255,0), 1);
   }
   
   
//            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//            state = KF.transitionMatrix*state + processNoise;
   
            imshow( "mouse kalman", img );
            code = (char)waitKey(10);
   
            if( code > 0 )
                break;
        }
 
    return 0;
} 