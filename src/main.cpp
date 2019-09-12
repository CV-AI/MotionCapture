#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Acquisition.hpp"
#include "DataProcess.hpp"
#include "Tracker.hpp"
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
    // initialize
    Tracker tracker;
	DataProcess dataProcess;
	bool status = true;
    // let the program know which camera to acquire image from
    
    cv::Mat image_LU, image_RU, image_RL, image_LL; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::WindowFlags window_type = cv::WINDOW_NORMAL;
    cv::namedWindow("Left_Upper", window_type);
    cv::namedWindow("Right_Upper", window_type);
    cv::namedWindow("Right_Lower", window_type);
    cv::namedWindow("Left_Lower", window_type);
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
	assert(dataProcess.numCameras == numCameras);
	std::cout << "Number of cameras detected: " << numCameras << "\n" << std::endl;
	assert(numCameras == NUM_CAMERAS);
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
            if (deviceSerialNumber=="18308397")
            {
                CameraIndex[i] = 0; // Left Upper
            }
            else if (deviceSerialNumber=="18308395")
            {
                CameraIndex[i] = 1; // Left Lower
            }
            else if (deviceSerialNumber == "18308399")
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
		TrackerParameters* trackerParaList = new TrackerParameters[NUM_CAMERAS];
		Tracker* trackerList = new Tracker[NUM_CAMERAS];
		for (int j = 0; j < NUM_CAMERAS; j++)
		{
			trackerParaList[j].trackerPtr = &trackerList[j];
			trackerParaList[j].camera_index = j;
			trackerParaList[j].tracker_type = ByDetection;
			paraList[j].pCam = camList.GetByIndex(j);
			paraList[j].cvImage = &tracker.ReceivedImages[CameraIndex[j]];
		}
		HANDLE* grabThreads = new HANDLE[NUM_CAMERAS];
		HANDLE* trackerThreads = new HANDLE[NUM_CAMERAS];


		
        // acquire images and do something
        // main part of this program
        //cv::setMouseCallback("Left_Upper", tracker.Mouse_getColor, 0); 
		bool first_time = true;
		int num_Acquisition = 0; // init tracker after some images to assure auto balance finished
		try
		{
			auto start_acquiring = std::chrono::high_resolution_clock::now();
			auto start_tracking = std::chrono::high_resolution_clock::now();
			auto finish_tracking = std::chrono::high_resolution_clock::now();
			auto stop_export = std::chrono::high_resolution_clock::now();
			auto stop_drawing = std::chrono::high_resolution_clock::now();
			while (status)
			{
				// acquire images
				start_acquiring = std::chrono::high_resolution_clock::now();
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
				
				start_tracking = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> time_acquiring = start_tracking - start_acquiring;
				std::cout << "Acquiring time on camera " << ": " << time_acquiring.count() << std::endl;


				// pCam = NULL;
				if (tracker.TrackerAutoIntialized)
				{
					memcpy(tracker.previousPosSet, tracker.currentPosSet, sizeof(tracker.currentPosSet));

					for (int j = 0; j < NUM_CAMERAS; j++)
					{
						// Start grab thread
					/*cout << "processing" << i << endl;*/
						trackerThreads[j] = CreateThread(nullptr, 0, UpdateTracker, &trackerParaList[j], 0, nullptr);
						assert(trackerThreads[j] != nullptr);
					}
					// Wait for all threads to finish
					WaitForMultipleObjects(NUM_CAMERAS,		// number of threads to wait for 
						trackerThreads,				// handles for threads to wait for
						TRUE,					// wait for all of the threads
						INFINITE				// wait forever
					);
					// Check thread return code for each camera
					for (int j = 0; j < NUM_CAMERAS; j++)
					{
						DWORD exitcode;

						BOOL rc = GetExitCodeThread(trackerThreads[j], &exitcode);
						if (!rc)
						{
							cout << "Handle error from GetExitCodeThread() returned for camera at index " << j << endl;
						}
						else if (!exitcode)
						{
							cout << "Grab thread for camera at index " << j << " exited with errors."
								"Please check onscreen print outs for error details" << endl;
						}
					}
					
					for (int i = 0; i < NUM_CAMERAS; i++)
					{
						tracker.RectifyMarkerPos(i);

						for (int marker_index = 0; marker_index < NUM_MARKERS; marker_index++)
						{
							//std::cout << "Camera " << i << " Marker " << marker_index << " " << tracker.currentPos[i][marker_index] << std::endl;
							cv::putText(tracker.ReceivedImages[i], to_string(i), cv::Point(50, 50), 1, 2, cv::Scalar(0, 255, 0), 2);
							cv::putText(tracker.ReceivedImages[i], to_string(marker_index), tracker.currentPos[i][marker_index], 1, 2, cv::Scalar(0, 255, 0), 2);
							cv::circle(tracker.ReceivedImages[i], tracker.currentPos[i][marker_index], 3, cv::Scalar(0, 0, 255), 2);
							// std::cout << i << marker_index << tracker.currentPos[i][marker_index] << std::endl;

						}
						for (int marker_set = 0; marker_set < NUM_MARKER_SET; marker_set++)
						{
							cv::rectangle(tracker.ReceivedImages[i], cv::Rect(tracker.currentPosSet[i][marker_set].x - tracker.detectWindowDimX / 2,
								tracker.currentPosSet[i][marker_set].y - tracker.detectWindowDimY / 2, tracker.detectWindowDimX, tracker.detectWindowDimY), cv::Scalar(255, 0, 0),2);
						}
					}
					finish_tracking = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double> time_tracking = finish_tracking - start_tracking;
					std::cout << "Time on tracking " << ": " << time_tracking.count() << std::endl;
					for (int camera_index = 0; camera_index < NUM_CAMERAS; camera_index++)
					{
						for (int marker_inex = 0; marker_inex < NUM_MARKERS; marker_inex++)
						{
							// 因为我们截取了一部分图像，所以计算位置之前要还原到原来的2048*2048的像素坐标系下的坐标
							dataProcess.points[camera_index][marker_inex] = tracker.currentPos[camera_index][marker_inex]
								+ dataProcess.offset[camera_index];
							/*cout << "offset" << dataProcess.offset[camera_index]<<endl;
							cout << "tracker pos" << tracker.currentPos[camera_index][marker_inex] << endl;
							cout << "dataprocess pos" << dataProcess.points[camera_index][marker_inex] << endl;*/
						}
					}
					dataProcess.exportGaitData();
					//Sleep(15); // sleep for milliseconds
					stop_export = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double> time_export = stop_export - finish_tracking;
					std::cout << "Time on export gait data: " << time_export.count() << std::endl;
					
				}

				if (/*tracker.getColors && */!tracker.TrackerAutoIntialized && num_Acquisition > 30
#ifdef TRANS_FRAME 
					&& dataProcess.GotWorldFrame
#endif
					)
				{
					cv::imshow("Left_Upper", tracker.ReceivedImages[0]);
					cv::imshow("Left_Lower", tracker.ReceivedImages[1]);
					cv::imshow("Right_Upper", tracker.ReceivedImages[2]);
					cv::imshow("Right_Lower", tracker.ReceivedImages[3]);
					tracker.InitTracker(ByDetection);
					//getchar();
				}
#ifdef TRANS_FRAME

				if (!dataProcess.GotWorldFrame && num_Acquisition > 15)
				{
					//dataProcess.GotWorldFrame = true;
					try
					{
						bool found = dataProcess.FindWorldFrame(tracker.ReceivedImages[0], tracker.ReceivedImages[1]);
						found = dataProcess.FindWorldFrame(tracker.ReceivedImages[2], tracker.ReceivedImages[3]) && found;
						if (found)
						{
							dataProcess.GotWorldFrame = true;
							cout << "Find world coordinate system succeed!" << endl;
						}
						else
						{
							dataProcess.GotWorldFrame = false;
							cout << "Faild find world coordinate system!\a\a\a" << endl;
						}
					}
					catch (cv::Exception& e)
					{
						std::cout << "OpenCV Error: during finding world frame: \n" << e.what() << endl;
					}
				}
#endif
				/*cv::imshow("Left_Upper", tracker.ReceivedImages[0]);
				cv::imshow("Left_Lower", tracker.ReceivedImages[1]);
				cv::imshow("Right_Upper", tracker.ReceivedImages[2]);
				cv::imshow("Right_Lower", tracker.ReceivedImages[3]);*/
				cv::Mat combine, combine1, combine2, combine3;
				cv::hconcat(tracker.ReceivedImages[2], tracker.ReceivedImages[0], combine1);
				cv::hconcat(tracker.ReceivedImages[3], tracker.ReceivedImages[1], combine2);
				cv::vconcat(combine1, combine2, combine);
				cv::imshow("CONCAT", combine);
				int key = cv::waitKey(1);
				if (key == 27)
				{
					status = false;
				}
				num_Acquisition += 1;
				stop_drawing = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> time_showing = stop_drawing - stop_export;
				std::cout << "Time on showing images: " << time_showing.count() << std::endl;
				std::chrono::duration<double> time_total = stop_drawing - start_acquiring;
				std::cout << "Total time: " << time_total.count() << std::endl;
				
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
		for (int j = 0; j < NUM_CAMERAS; j++)
		{
			CloseHandle(trackerThreads[j]);
		}

		// Delete array pointer
		delete[] paraList;
		delete[] trackerParaList;
		delete[] trackerThreads;
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
