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

#define TRANS_FRAME

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
	// open Ads
	dataProcess.nPort = AdsPortOpen();
	dataProcess.AdsOpened = true;
	dataProcess.pAddr->netId.b[0] = 172;
	dataProcess.pAddr->netId.b[1] = 18;
	dataProcess.pAddr->netId.b[2] = 130;
	dataProcess.pAddr->netId.b[3] = 124;
	dataProcess.pAddr->netId.b[4] = 1;
	dataProcess.pAddr->netId.b[5] = 1;

	/*dataProcess.nErr = AdsGetLocalAddress(dataProcess.pAddr);
	if (dataProcess.nErr)
	{
		cerr << "Error: AdsGetLocalAddress: " << dataProcess.nErr << "\n";
	}*/
	dataProcess.pAddr->port = 851;
	dataProcess.nErr = AdsSyncReadWriteReq(dataProcess.pAddr, ADSIGRP_SYM_HNDBYNAME, 0x0, sizeof(dataProcess.lHdlVar2), &dataProcess.lHdlVar2, sizeof(dataProcess.szVar2), dataProcess.szVar2);
	if (dataProcess.nErr) 
	{	
		dataProcess.AdsOpened = false;
		cerr << "Error: AdsSyncReadWriteReq: " << dataProcess.nErr << "\n"; 
	}
    // let the program know which camera to acquire image from
    
    cv::Mat image_LU, image_RU, image_RL, image_LL; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	auto window_type = 0; 
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
	
	std::cout << "Number of cameras detected: " << numCameras << "\n" << std::endl;
	assert(numCameras == NUM_CAMERAS);
	// set current process as high priority (second highest, the highest is Real time)
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    CameraPtr pCam = nullptr;
    try 
    {
        // Give each camera index
        for (unsigned int i = 0; i < NUM_CAMERAS; i++)
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
            if (deviceSerialNumber=="18308396")
            {
                CameraIndex[i] = 0; // Left Upper
            }
            else if (deviceSerialNumber=="18308397")
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
		for (unsigned int i = 0; i < NUM_CAMERAS; i++)
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
				pCam = nullptr;
				std::cout << "Press Any Key to exit..." << endl;
				char _ = getchar();
				return -1;
			}
		}
		//cv::waitKey(1500); // 等待1.5s,确保相机自动白平衡已经完成
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
		//cv::setMouseCallback("Left_Upper", tracker.Mouse_getColor, 0); 

		// acquire images and do something
		// main part of this program
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
							cout << "Tracking thread for camera at index " << j << " exited with errors."
								"Please check onscreen print outs for error details" << endl;
						}
					}
					
					for (int i = 0; i < NUM_CAMERAS; i++)
					{
						tracker.RectifyMarkerPos(i);
						cv::putText(tracker.ReceivedImages[i], to_string(i), cv::Point(50, 50), 1, 4, cv::Scalar(0, 255, 85), 2);
						for (int marker_set = 0; marker_set < NUM_MARKER_SET; marker_set++)
						{
							// 写出标记点对的编号
							cv::putText(tracker.ReceivedImages[i], to_string(marker_set), cv::Point(tracker.currentPosSet[i][marker_set].x - tracker.detectWindowDim[i][marker_set].x / 2,
								tracker.currentPosSet[i][marker_set].y - tracker.detectWindowDim[i][marker_set].y / 2 -10), 1, 2, cv::Scalar(0, 255, 85), 2);
							// 画出箭头
							cv::arrowedLine(tracker.ReceivedImages[i], tracker.currentPos[i][2*marker_set], tracker.currentPos[i][2*marker_set+1], cv::Scalar(0,100,255),2, 8, 0, 0.3);
							// 画出检测窗
							cv::rectangle(tracker.ReceivedImages[i], cv::Rect(tracker.currentPosSet[i][marker_set].x - tracker.detectWindowDim[i][marker_set].x / 2,
								tracker.currentPosSet[i][marker_set].y - tracker.detectWindowDim[i][marker_set].y / 2, tracker.detectWindowDim[i][marker_set].x, tracker.detectWindowDim[i][marker_set].y), cv::Scalar(255, 102, 0),2);
						}
					}
					finish_tracking = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double> time_tracking = finish_tracking - start_tracking;
					std::cout << "Time on tracking " << ": " << time_tracking.count() << std::endl;
					for (int camera_index = 0; camera_index < NUM_CAMERAS; camera_index++)
					{
						for (int marker_inex = 0; marker_inex < NUM_MARKERS; marker_inex++)
						{
							if (_PRINT_PROCESS)
							{
								std::cout << "Tracker points Camera " << camera_index << " marker " << marker_inex << tracker.currentPos[camera_index][marker_inex] << std::endl;
							}
							// 因为我们截取了一部分图像，所以计算位置之前要还原到原来的2048*2048的像素坐标系下的坐标
							dataProcess.points[camera_index][marker_inex] = tracker.currentPos[camera_index][marker_inex]
								+ cv::Point2f(offset[camera_index]);
						}
					}
					dataProcess.exportGaitData();
					stop_export = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double> time_export = stop_export - finish_tracking;
					std::cout << "Time on export gait data: " << time_export.count() << std::endl;
				}

				if (/*tracker.getColors && */!tracker.TrackerAutoIntialized
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
					cv::destroyWindow("Left_Upper");
					cv::destroyWindow("Left_Lower");
					cv::destroyWindow("Right_Upper");
					cv::destroyWindow("Right_Lower");
				}
#ifdef TRANS_FRAME
			
				if (!dataProcess.GotWorldFrame)
				{
					try
					{
					
						bool found = dataProcess.FindWorldFrame(tracker.ReceivedImages);
						if (found)
						{
							cout << "Find world coordinate system succeed!" << endl;
						}
						else
						{
							cout << "Faild to find world coordinate system!\a\a\a" << endl;
						}
					}
					catch (cv::Exception& e)
					{
						std::cout << "OpenCV Error: during finding world frame: \n" << e.what() << endl;
					}
				}
#endif
				cv::Mat combine, combine1, combine2;
				cv::hconcat(tracker.ReceivedImages[2], tracker.ReceivedImages[0], combine1);
				cv::hconcat(tracker.ReceivedImages[3], tracker.ReceivedImages[1], combine2);
				cv::vconcat(combine1, combine2, combine);
				cv::imshow("CONCAT", combine);
				int key = cv::waitKey(1);
				if (key == 27) // 按下Esc键退出程序
				{
					status = false;
				}
				stop_drawing = std::chrono::high_resolution_clock::now();
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
    }
	pCam = nullptr; // 一定要设为nullptr, 否则在system->ReleaseInstance()时会出错

    // Clear camera list before releasing system
    camList.Clear();
    system->ReleaseInstance();
	// close Ads
	dataProcess.nErr = AdsPortClose();
	if (dataProcess.nErr) cerr << "Error: AdsPortClose: " << dataProcess.nErr << "\n";
    return 0;
}
