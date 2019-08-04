#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Acquisition.hpp"
#include "DataProcess.h"
#include "Tracker.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>
#include <Windows.h>
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
    
    cv::Mat image_LU, image_RU, image_RL, image_LL; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
    cv::namedWindow("Left_Upper",0);
    cv::namedWindow("Right_Upper",0);
    cv::namedWindow("Right_Lower", 0);
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
	int CameraIndex[4] = { 0,1,2,3 };
	dataProcess.numCameras = tracker.numCameras = numCameras;
	assert(numCameras % 2 == 0, "Number of cameras not correct, must be multiple of 2.");
	
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
	// set current process as high priority (second highest, the highest is Real time)
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    CameraPtr pCam = NULL;
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
            if (deviceSerialNumber=="18308395")
            {
                CameraIndex[i] = 0; // Left Upper
            }
            else if (deviceSerialNumber=="18308397")
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
		AcquisitionParameters* paraList = new AcquisitionParameters[numCameras];
		TrackerParameters* trackerParaList = new TrackerParameters[NUM_MARKERS];
		Tracker* trackerList = new Tracker[NUM_MARKERS];
		for (int j = 0; j < NUM_MARKERS; j++)
		{
			trackerParaList[j].trackerPtr = &trackerList[j];
			trackerParaList[j].marker_index = j;
			trackerParaList[j].tracker_type = ByDetection;
		}
#if defined(_WIN32)
		HANDLE* grabThreads = new HANDLE[numCameras];
		HANDLE* trackerThreads = new HANDLE[NUM_MARKERS];
#else
		pthread_t* grabThreads = new pthread_t[numCameras];
		pthread_t* trackerThreads = new pthread_t[NUM_MARKERS];
#endif

		
        // acquire images and do something
        // main part of this program
        cv::setMouseCallback("Left_Upper", tracker.Mouse_getColor, 0);
		bool first_time = true;
		int num_Acquisition = 0; // init tracker after some images to assure auto balance finished
        while(status)
        {
            // acquire images
			auto start = std::chrono::high_resolution_clock::now();
			for (unsigned int i = 0; i < numCameras; i++)
			{
				paraList[i].pCam = camList.GetByIndex(i);
				paraList[i].cvImage = &tracker.ReceivedImages[CameraIndex[i]];
				// Start grab thread
#if defined(_WIN32)
				/*cout << "processing" << i << endl;*/
				grabThreads[i] = CreateThread(nullptr, 0, AcquireImages, &paraList[i], 0, nullptr);
				assert(grabThreads[i] != nullptr);
#else
				int err = pthread_create(&(grabThreads[i]), nullptr, &AcquireImages, &paraList[i]);
				assert(err == 0);
#endif
			}
#if defined(_WIN32)
			// Wait for all threads to finish
			WaitForMultipleObjects(numCameras,		// number of threads to wait for 
				grabThreads,				// handles for threads to wait for
				TRUE,					// wait for all of the threads
				INFINITE				// wait forever
			);
			// Check thread return code for each camera
			for (unsigned int i = 0; i < numCameras; i++)
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

#else
			for (unsigned int i = 0; i < camListSize; i++)
			{
				// Wait for all threads to finish
				void* exitcode;
				int rc = pthread_join(grabThreads[i], &exitcode);
				if (rc != 0)
				{
					cout << "Handle error from pthread_join returned for camera at index " << i << endl;
				}
				else if ((int)(intptr_t)exitcode == 0)// check thread return code for each camera
				{
					cout << "Grab thread for camera at index " << i << " exited with errors."
						"Please check onscreen print outs for error details" << endl;
				}
			}
#endif
			auto stop = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed_seconds = stop - start;
			std::cout << "Acquiring time on camera " << ": " << elapsed_seconds.count() << std::endl;
			
			
            // pCam = NULL;
			auto start_processing = std::chrono::high_resolution_clock::now();
			if (tracker.TrackerAutoIntialized)
			{	
				memcpy(tracker.previousPos, tracker.currentPos, sizeof(tracker.currentPos));

				for (int j = 0; j < NUM_MARKERS; j++)
				{
					// Start grab thread
				/*cout << "processing" << i << endl;*/
					trackerThreads[j] = CreateThread(nullptr, 0, UpdateTracker, &trackerParaList[j], 0, nullptr);
					assert(trackerThreads[j] != nullptr);
				}
				// Wait for all threads to finish
				WaitForMultipleObjects(NUM_MARKERS,		// number of threads to wait for 
					trackerThreads,				// handles for threads to wait for
					TRUE,					// wait for all of the threads
					INFINITE				// wait forever
				);
				// Check thread return code for each camera
				for (int j = 0; j < NUM_MARKERS; j++)
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
				for (int i = 0; i < numCameras; i++)
				{
					tracker.RectifyMarkerPos(i);
					for (int marker_index = 0; marker_index < NUM_MARKERS; marker_index++)
					{
						std::cout << i << marker_index << tracker.currentPos[i][marker_index] << std::endl;
						cv::circle(tracker.ReceivedImages[i], tracker.currentPos[i][marker_index], 3, cv::Scalar(0, 0, 255),3);
						cv::rectangle(tracker.ReceivedImages[i], cv::Rect(tracker.currentPos[i][marker_index].x - tracker.detectWindowDimX / 2,
							tracker.currentPos[i][marker_index].y - tracker.detectWindowDimY / 2, tracker.detectWindowDimX, tracker.detectWindowDimY), cv::Scalar(255, 0, 0));
					}
				}
				auto track_processing = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed_seconds_processing = track_processing - start_processing;
				std::cout << "Time on tracking " << ": " << elapsed_seconds_processing.count() << std::endl;
				memcpy(dataProcess.points, tracker.currentPos, sizeof(tracker.currentPos));
				dataProcess.exportGaitData();
				auto stop_export = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> seconds_export = stop_export - track_processing;
				std::cout << "Time on export gait data: " << seconds_export.count() << std::endl;
			}
			
			if (/*tracker.getColors && */!tracker.TrackerAutoIntialized && dataProcess.GotWorldFrame)
			{
				tracker.InitTracker(ByDetection);
				//getchar();
			}
			if (!dataProcess.GotWorldFrame && num_Acquisition > 15)
			{
				dataProcess.GotWorldFrame = true;
				/*try
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
						cout << "Find world coordinate system failed!\a\a\a" << endl;
					}
				}
				catch (cv::Exception& e)
				{
					std::cout << "OpenCV Error: during finding world frame: \n" << e.what() << endl;
				}*/
			}
			
			cv::imshow("Left_Upper", tracker.ReceivedImages[0]);
			cv::imshow("Left_Lower", tracker.ReceivedImages[1]);
			cv::imshow("Right_Upper", tracker.ReceivedImages[2]);
			cv::imshow("Right_Lower", tracker.ReceivedImages[3]);
			int key = cv::waitKey(1);
			if (key == 27)
			{
				status = false;
			}
			num_Acquisition += 1;
        }
		// Clear CameraPtr array and close all handles
		for (unsigned int i = 0; i < numCameras; i++)
		{    
			CloseHandle(grabThreads[i]);
		}
		for (int j = 0; j < NUM_MARKERS; j++)
		{
			CloseHandle(trackerThreads[j]);
		}

		// Delete array pointer
		delete[] paraList;
		delete[] trackerParaList;
		delete[] trackerThreads;
		delete[] grabThreads;
        cv::destroyAllWindows();
		pCam = NULL;
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
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);
        pCam->EndAcquisition();
		ResetExposure(pCam);
		ResetTrigger(pCam);
        pCam->DeInit();
    }
    //pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    getchar();
    return true;
}
