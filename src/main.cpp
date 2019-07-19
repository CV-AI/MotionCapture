#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Acquisition.hpp"
#include "DataProcess.h"
#include "operation.h"
#include "TemplateMatch.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{   
    // initialize
    // operation Operation;
	// TemplateMatch templ;
	// DataProcess dataProcess;
    int result = 0;
    cv::Mat image_l, image_r;
    cv::namedWindow("Left",0);
    cv::namedWindow("Right",0);
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    // Print Spinnaker library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build << endl << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
  
    unsigned int numCameras = camList.GetSize();
    
    std::cout << "Number of cameras detected: " << numCameras << endl << endl;
    
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        std::cout << "Not enough cameras!" << endl;
        std::cout << "Done! Press Enter to exit..." << endl;
        getchar();
        
        return -1;
    }
    CameraPtr pCam = NULL;
    try 
    {
        // Config each camera
        for (unsigned int i = 0; i < numCameras; i++)
        {
            // Select camera
            pCam = camList.GetByIndex(i);

            std::cout << endl << "Configuring Camera " << i << "..." << endl;
            if(pCam->IsInitialized() || pCam->IsStreaming())
            {
                pCam->EndAcquisition();
                pCam->DeInit();
                cout <<"Error: during Init Camera(camera is already IsInitialized"<<endl;
            }
            pCam->Init();  
            pCam->BeginAcquisition();
            // Run example
            result = result | ConfigCamera(pCam);
            
            // pCam->BeginAcquisition();
            std::cout << "Camera " << i << " config complete..." << endl << endl;
            // pCam =NULL;
        }
        // acquire images and do something
        bool ACQUISITION = true;
        while(true)
        {
            for (unsigned int i = 0; i < numCameras; i++)
            {
                // Select camera
                pCam = camList.GetByIndex(i);
                if(i==0)
                {
                    auto start = std::chrono::system_clock::now();
                    image_l = AcquireImages(pCam);
                    imwrite("image_l.jpg", image_l);
                    
                    auto end = std::chrono::system_clock::now();
                    cout<<image_l.channels()<<" "<<image_l.cols <<endl;
                    std::chrono::duration<double> elapsed_seconds = end-start;
                    cout << "acquire time: "<<elapsed_seconds.count()<<endl;
                    cv::imshow("Left", image_l);
                    int key = cv::waitKey(1);
                    if ( key == 27) // press "Esc" to stop
                    {ACQUISITION = false;}

                }
                else
                {
                    cout<<"right"<< endl;
                    image_r = AcquireImages(pCam);
                    cv::imshow("Right", image_r);
                    cv::imwrite("Right.jpg", image_r);
                    int key = cv::waitKey(1);
                    if ( key == 27) // press "Esc" to stop
                    {ACQUISITION = false;}
                }
                // pCam = NULL;
            }
        }
    }
    // sometimes AcquireImages may throw cv::Exception or Spinnaker::Exception
    // using try catch makes sure we EndAcquisition for each camera
    catch(Spinnaker::Exception &e)
    {
        std::cout << "Error: during print Device information" << e.what() << endl;
        result = -1;
    }
    catch (cv::Exception& e)  
    {      
        std::cout << "Error: during print Device information" << e.what() << endl;
        result = -1;
    }  
    // EndAcquisition
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);
        pCam->EndAcquisition();
        pCam->DeInit();
    }
    //pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    std::cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();
    return result;
}
