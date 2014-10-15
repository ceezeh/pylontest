// Grab.cpp
/*
Note: Before getting started, Basler recommends reading the Programmer's Guide topic
in the pylon C++ API documentation that gets installed with pylon.
If you are upgrading to a higher major version of pylon, Basler also
strongly recommends reading the Migration topic in the pylon C++ API documentation.

This sample illustrates how to grab and process images using the CInstantCamera class.
The images are grabbed and processed asynchronously, i.e.,
while the application is processing a buffer, the acquisition of the next buffer is done
in parallel.

The CInstantCamera class uses a pool of buffers to retrieve image data
from the camera device. Once a buffer is filled and ready,
the buffer can be retrieved from the camera object for processing. The buffer
and additional image data are collected in a grab result. The grab result is
held by a smart pointer after retrieval. The buffer is automatically reused
when explicitly released or when the smart pointer object is destroyed.
*/
#include <ros/ros.h>
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#include <pylon/PylonGUI.h>
#endif

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
#include <pylon/ImageFormatConverter.h>

  using namespace cv;

  // Namespace for using pylon objects.
  using namespace Pylon;


  // Namespace for using cout.
  using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "grab");

          ros::NodeHandle n;

          ros::Rate loop_rate(10);

// The exit code of the sample application.
int exitCode = 0;

// Automagically call PylonInitialize and PylonTerminate to ensure
// the pylon runtime system is initialized during the lifetime of this object.
Pylon::PylonAutoInitTerm autoInitTerm;


CGrabResultPtr ptrGrabResult;
namedWindow("CV_Image",WINDOW_AUTOSIZE);
try
{

    CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
    cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
     camera.Open();

    GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
     GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
     Mat cv_img(width->GetValue(), height->GetValue(), CV_8UC3);

    camera.StartGrabbing();
    CPylonImage image;
    CImageFormatConverter fc;
     fc.OutputPixelFormat = PixelType_RGB8packed;

    while(camera.IsGrabbing()){
        camera.RetrieveResult( 500, ptrGrabResult, TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded()){
                 fc.Convert(image, ptrGrabResult);

                cv_img = cv::Mat(ptrGrabResult->GetHeight(),     ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)image.GetBuffer());
                imshow("CV_Image",cv_img);
                  waitKey(1);
                if(waitKey(30)==27){
                      camera.StopGrabbing();
                }
        }
        ros::spinOnce();

                        loop_rate.sleep();
    }

}

catch (GenICam::GenericException &e)
{
    // Error handling.
    cerr << "An exception occurred." << endl
    << e.GetDescription() << endl;
    exitCode = 1;
}

// Comment the following two lines to disable waiting on exit
cerr << endl << "Press Enter to exit." << endl;
while( cin.get() != '\n');

return exitCode;
 }
