#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Acquisition.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>
#include <Windows.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define TRANS_FRAME

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{   
   
	bool status = true;
	TRACKING = false;
    // let the program know which camera to acquire image from
    
    cv::Mat image_LU, image_RU, image_RL, image_LL; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	auto window_type = 0;
	cv::namedWindow("CONCAT", window_type);
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    // Print Spinnaker library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build << "\n" << std::endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
  
    unsigned int numCameras = camList.GetSize();
	int CameraIndex[4] = { 0,1,2,3 };
	std::cout << "Number of cameras detected: " << numCameras << "\n" << std::endl;
	// set current process as high priority (second highest, the highest is Real time)
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    CameraPtr pCam = nullptr;
    try 
    {
        // Give each camera index
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
			if (deviceSerialNumber == "18308396")
			{
				CameraIndex[i] = 0; // Left Upper
			}
			else if (deviceSerialNumber == "18308397")
			{
				CameraIndex[i] = 1; // Left Lower
			}
			else if (deviceSerialNumber == "18308400")
			{
				CameraIndex[i] = 2; // Right Upper               
			}
			else if (deviceSerialNumber == "18308393")
			{
				CameraIndex[i] = 3; // Right Lower
			}
			cout << "Camera Index on " << i << " is " << CameraIndex[i] << endl;
        }
		for (unsigned int i = 0; i < numCameras; i++)
		{
			pCam = camList.GetByIndex(i);
			std::cout << endl << "Configuring Camera " << i << "..." << endl;
			pCam->Init();
			if (pCam->IsInitialized())
			{
				std::cout << "Camera " << i << " initialized successly" << endl;
			}
			// Config camera
			status = status && ConfigCamera(pCam, CameraIndex[i]);
			pCam->BeginAcquisition();
			if (status)
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
		}
		// Create an array of CameraPtrs. This array maintenances smart pointer's reference
		// count when CameraPtr is passed into grab thread as void pointer
		AcquisitionParameters* paraList = new AcquisitionParameters[NUM_CAMERAS];
		cv::Mat images[NUM_CAMERAS];
		HANDLE* grabThreads = new HANDLE[NUM_CAMERAS];
		for (int j = 0; j < NUM_CAMERAS; j++)
		{
			paraList[j].pCam = camList.GetByIndex(j);
			paraList[j].cvImage = &images[CameraIndex[j]];
		}
        // acquire images and do something
        // main part of this program
        //cv::setMouseCallback("Left_Upper", tracker.Mouse_getColor, 0); 
		bool first_time = true;
		int num_Acquisition = 0; // init tracker after some images to assure auto balance finished
		try
		{
			while (status)
			{

				// acquire images
				for (unsigned int i = 0; i < NUM_CAMERAS; i++)
				{
					// Start grab thread
					grabThreads[i] = CreateThread(nullptr, 0, AcquireImages, &paraList[i], 0, nullptr);
					assert(grabThreads[i] != nullptr);
				}
				// Wait for all threads to finish
				WaitForMultipleObjects(NUM_CAMERAS,		// number of threads to wait for 
					grabThreads,				// handles for threads to wait for
					TRUE,					// wait for all of the threads
					INFINITE				// wait forever
				);
				// Check thread return code for each camera
				for (unsigned int i = 0; i < NUM_CAMERAS; i++)
				{
					DWORD exitcode;

					BOOL rc = GetExitCodeThread(grabThreads[i], &exitcode);
					if (!rc)
					{
						cout << "Handle error from GetExitCodeThread() returned for camera at index " << i << endl;
					}
					else if (!exitcode)
					{
						cout << "Grab thread for camera at index " << i << " exited with errors."
							"Please check onscreen print outs for error details" << endl;
					}
				}
				cv::Mat combine, combine1, combine2;
				cout << images[0].size() << images[1].size() << endl;
				cout << images[2].size() << images[3].size() << endl;
				cv::hconcat(images[2], images[0], combine1);
				cv::hconcat(images[3], images[1], combine2);
				cv::vconcat(combine1, combine2, combine);
				cv::resize(combine, combine, cv::Size(1024, 1024));
				cv::imshow("CONCAT", combine);

				int key = cv::waitKey(1);
				if (key == 27) // esc
				{
					status = false;
				}
				if (key == 32) // space
				{

					string string0 = "D:\\MotionCapture\\build\\LeftUpper\\" + to_string(num_Acquisition) + ".jpg";   //"LeftUpper/"
					string string1 = "D:\\MotionCapture\\build\\LeftLower\\" + to_string(num_Acquisition) + ".jpg";
					string string2 = "D:\\MotionCapture\\build\\RightUpper\\" + to_string(num_Acquisition) + ".jpg";
					string string3 = "D:\\MotionCapture\\build\\RightLower\\" + to_string(num_Acquisition) + ".jpg";
					cv::imwrite(string0, images[0]);
					cv::imwrite(string1, images[1]);
					cv::imwrite(string2, images[2]);
					cv::imwrite(string3, images[3]);
					std::cout << "Add image pairs " << num_Acquisition << std::endl;
					num_Acquisition++;
				}
				
			}
		}
		catch (Spinnaker::Exception& e)
		{
			std::cout << "Spinnaker Error: during tracking: \n" << e.what() << endl;
		}
		catch (cv::Exception& e)
		{
			std::cout << "OpenCV Error: during tracking: \n" << e.what() << endl;
		}
		// Clear CameraPtr array and close all handles
		for (unsigned int i = 0; i < NUM_CAMERAS; i++)
		{    
			CloseHandle(grabThreads[i]);
		}

		// Delete array pointer
		delete[] paraList;
		delete[] grabThreads;
        cv::destroyAllWindows();
		pCam = nullptr;
    }
    // sometimes AcquireImages may throw cv::Exception or Spinnaker::Exception
    // using try catch makes sure we EndAcquisition for each camera
    catch(Spinnaker::Exception &e)
    {
        std::cout << "Spinnaker Error: during Running Camera\n" << e.what() << endl;
    }
    catch (cv::Exception& e)  
    {      
        std::cout << "OpenCV Error: during Running Camera: \n" << e.what() << endl;
    }  
    // EndAcquisition
    for (unsigned int i = 0; i < NUM_CAMERAS; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);
        pCam->EndAcquisition();
		ResetExposure(pCam);
		ResetTrigger(pCam);
        pCam->DeInit();
		pCam = nullptr; // 一定要设为nullptr, 否则在system->ReleaseInstance()时会出错
    }
    //pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();
    system->ReleaseInstance();
    
    return true;
}
