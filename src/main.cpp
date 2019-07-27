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
	tracker.numCameras = numCameras;
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
                CameraIndex[i] = 0; // image_LU
            }
            else if (deviceSerialNumber=="18308397")
            {
                CameraIndex[i] = 1; // image_LL
            }
            else if (deviceSerialNumber == "18308393")
            {
                CameraIndex[i] = 2; // image_RU                
            }
            else if (deviceSerialNumber == "18308391")
            {
                CameraIndex[i] = 3; // image_RL
            }
            std::cout << endl << "Configuring Camera " << i << "..." << endl;
            pCam->Init();  
            if(pCam->IsInitialized())
            {
                std::cout <<"Camera " << i <<" initialized successly"<<endl;
            }
            // Config camera
            status = status && ConfigCamera(pCam);
			pCam->BeginAcquisition();
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
                // 注意目前pCam指向的是哪个相机
                pCam = camList.GetByIndex(i);
				auto start = std::chrono::high_resolution_clock::now();
                switch(CameraIndex[i])
                {
					case 0: 
						tracker.ReceivedImages[0] = AcquireImages(pCam);
						cv::resize(tracker.ReceivedImages[0], image_LU, cv::Size(512, 512));
						tracker.image = image_LU.clone(); // tracker.image is used for select color
						cv::imshow("Left_Upper", image_LU); break; // show resized images to save time
					case 1: 
						tracker.ReceivedImages[1] = AcquireImages(pCam); 
						cv::resize(tracker.ReceivedImages[1], image_LL, cv::Size(512, 512));
						cv::imshow("Left_Lower", image_LL); break;
					case 2: 
						tracker.ReceivedImages[2] = AcquireImages(pCam);
						cv::resize(tracker.ReceivedImages[2], image_RU, cv::Size(512, 512)); 
						cv::imshow("Right_Upper", image_RU); break;
					case 3: 
						tracker.ReceivedImages[3] = AcquireImages(pCam);
						cv::resize(tracker.ReceivedImages[3], image_RL, cv::Size(512, 512)); 
						cv::imshow("Right_Lower", image_RL); break;
                }
				auto stop = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed_seconds = stop - start;
				std::cout << "Acquiring time on camera " << CameraIndex[i]<<": " <<elapsed_seconds.count() <<std::endl;
                int key = cv::waitKey(1);
                if ( key == 27) // press "Esc" to stop
                {status = false;}
                // pCam = NULL;
            }
			// 这两者顺序不要弄反，否则初始化后会立即开始跟踪，这时图片还未更新，如果使用
            if(tracker.TrackerIntialized)
            {
                memcpy(tracker.previousPos, tracker.currentPos, sizeof(tracker.currentPos));
                tracker.UpdateTracker(tracker.ByDetection);
                dataProcess.exportGaitData();
            }
			if (first_time && tracker.getColors)
			{
				tracker.InitTracker(tracker.ByDetection);
				dataProcess.exportGaitData();
			}
        }
        cv::destroyAllWindows();
		pCam = NULL;
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
		ResetExposure(pCam);
        pCam->DeInit();
    }
    //pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    std::cout<< "Done! Press Enter to exit..." << endl;
    getchar();
    return status;
}
