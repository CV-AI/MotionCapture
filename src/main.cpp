#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Acquisition.hpp"
#include "DataProcess.h"
#include "Tracker.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>
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
    Tracker tracker;
	DataProcess dataProcess;
    bool status = true;
    // let the program know which camera to acquire image from
    int CameraIndex[4];
    cv::Mat image_LU, image_RU, image_RL, image_LL; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
    cv::namedWindow("Left_Upper",0);
    cv::namedWindow("Right_Upper",0);
    cv::namedWindow("Right_Lower:", 0);
    cv::namedWindow("Left_Lower", 0);
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
            gcstring deviceSerialNumber;
            if (pCam->TLDevice.DeviceSerialNumber != NULL && pCam->TLDevice.DeviceSerialNumber.GetAccessMode() == RO)
            {
                deviceSerialNumber =  pCam->TLDevice.DeviceSerialNumber.ToString();
            }
            else
            {
                cout << "Error: DeviceSerialNumber unavailable" << endl;
            }
            if (deviceSerialNumber=="18308395")
            {
                CameraIndex[0] = i; // image_LU
            }
            else if (deviceSerialNumber=="18308397")
            {
                CameraIndex[1] = i; // image_LL
            }
            else if (deviceSerialNumber == "18308399")
            {
                CameraIndex[2] = i; // image_RU                
            }
            else if (deviceSerialNumber == "18308391")
            {
                CameraIndex[3] = i; // image_RL
            }
            std::cout << endl << "Configuring Camera " << i << "..." << endl;
            pCam->Init();  
            // if not IsInitialized, close system 
            // if(!pCam->IsInitialized())
            // {
            //     // Clear camera list before releasing system
            //     camList.Clear();
            //     // Release system
            //     system->ReleaseInstance();
            //     std::cout << "Error: can't initialize camera" << endl;
            //     std::cout << "Press Enter to exit..." << endl;
            //     getchar();
            //     return -1;
            // }
            if(pCam->IsInitialized())
            {
                std::cout <<"Camera " << i <<" initialized successly"<<endl;
            }
            // Run example
            status = status && ConfigCamera(pCam);
            
            if(status)
            {
                std::cout << "Camera " << i << " config complete..." << "\n" << endl;
            }
            else
            {
                std::cout << "Camera " << i << " config failed..." << "\n" << endl;
                // Clear camera list before releasing system
                camList.Clear();
                // Release system
                system->ReleaseInstance();
                std::cout << "Press Any Key to exit..." << endl;
                getchar();
                return -1;
            }
            // pCam =NULL;
        }
        // acquire images and do something
        // main part of this program
        bool first_time = true;
        cv::setMouseCallback("Left_Upper", tracker.Mouse_getColor, 0);
        while(status)
        {
            // acquire images
            for (unsigned int i = 0; i < numCameras; i++)
            {
                // 注意目前pCam指向的其实是CameraIndex[i]所指的相机，也就是在Config阶段确定的相机
                // note which camera pCam is pointed into is determined by CameraIndex[i]
                pCam = camList.GetByIndex(CameraIndex[i]);
                switch(CameraIndex[i])
                {
                    case 0: tracker.ReceivedImages[0] = AcquireImages(pCam); cv::imshow("Left_Upper", image_LU);break;
                    case 1: tracker.ReceivedImages[1] = AcquireImages(pCam); cv::imshow("Left_Lower", image_LL);break;
                    case 2: tracker.ReceivedImages[2] = AcquireImages(pCam); cv::imshow("Right_Upper", image_RU);break;
                    case 3: tracker.ReceivedImages[3] = AcquireImages(pCam); cv::imshow("Right_Lower", image_RL);break;
                }
                int key = cv::waitKey(1);
                if ( key == 27) // press "Esc" to stop
                {status = false;}
                // pCam = NULL;
            }
            if(first_time && tracker.getColors)
            {
                tracker.InitTracker(tracker.ByDetection);
                dataProcess.exportGaitData();
            }
            if(tracker.TrackerIntialized)
            {
                memcpy(tracker.previousPos, tracker.currentPos, sizeof(tracker.currentPos));
                tracker.UpdateTracker(tracker.ByDetection);
                dataProcess.exportGaitData();
            }
        }
        cv::destroyAllWindows();
    }
    // sometimes AcquireImages may throw cv::Exception or Spinnaker::Exception
    // using try catch makes sure we EndAcquisition for each camera
    catch(Spinnaker::Exception &e)
    {
        std::cout << "Spinnaker Error: during Running Camera" << e.what() << endl;
    }
    catch (cv::Exception& e)  
    {      
        std::cout << "CVError: during Running Camera" << e.what() << endl;
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
    return status;
}
