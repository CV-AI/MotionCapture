/**
 *  @example Acquisition.cpp
 *
 *  @brief Acquisition.cpp shows how to acquire images. It relies on 
 *  information provided in the Enumeration example. Also, check out the
 *  ExceptionHandling and NodeMapInfo examples if you haven't already. 
 *  ExceptionHandling shows the handling of standard and Spinnaker exceptions
 *  while NodeMapInfo explores retrieving information from various node types.  
 *
 *  This example touches on the preparation and cleanup of a camera just before
 *  and just after the acquisition of images. Image retrieval and conversion,
 *  grabbing image data, and saving images are all covered as well.
 *
 *  Once comfortable with Acquisition, we suggest checking out 
 *  AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi. 
 *  AcquisitionMultipleCamera demonstrates simultaneously acquiring images from 
 *  a number of cameras, NodeMapCallback serves as a good introduction to 
 *  programming with callbacks and events, and SaveToAvi exhibits video creation.
 */
 
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef _WIN32
#include <pthread.h>
#endif

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

enum triggerType
{
	SOFTWARE,
	HARDWARE
};
const triggerType chosenTrigger = SOFTWARE;
const int64_t offsetX = 220;
const int64_t offsetY = 220;
int64_t height = 1440;
int64_t width = 1440;
class AcquisitionParameters
{
public:
	AcquisitionParameters()
	{
		pCam = NULL;
		cvImage = &cv::Mat(height, width, CV_8UC3);
	}
	CameraPtr pCam = NULL;
	cv::Mat* cvImage;
};
#ifdef _DEBUG
// Disables heartbeat on GEV cameras(GigE Vison) so debugging does not incur timeout errors
int DisableHeartbeat(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << "\n" << endl;
    //
    // Write to boolean node controlling the camera's heartbeat
    // 
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents 
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera. 
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera 
    // to its default settings. 
    // 

    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) && !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << "\n" << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << "\n" << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if ( !IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat) )
            {
                cout << "Unable to disable heartbeat on camera. Aborting..." << "\n" << endl;
                return -1;
            }
            ptrDeviceHeartbeat->SetValue(true);
            cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
            cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << "\n" << endl;
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << "\n" << endl;
        }
    }
    return 0;
}
#endif
cv::Mat ConvertToCVmat(ImagePtr spinImage);
cv::Mat AcquireImages(CameraPtr  pCam);
bool ConfigCamera(CameraPtr pCam);
int ResetExposure(CameraPtr pCam);
int PrintDeviceInfo(INodeMap & nodeMap);
/*
 * This function shows how to convert between Spinnaker ImagePtr container to CVmat container used in OpenCV.
*/
cv::Mat ConvertToCVmat(ImagePtr spinImage)
{
    unsigned int XPadding = spinImage->GetXPadding();
    unsigned int YPadding = spinImage->GetYPadding();
    unsigned int rowsize = spinImage->GetWidth();
    unsigned int colsize = spinImage->GetHeight();
    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_MAKETYPE(CV_8U, spinImage->GetNumChannels()), spinImage->GetData(), spinImage->GetStride());
}
// This function acquires images from a device.
#if defined (_WIN32)
DWORD WINAPI AcquireImages(LPVOID lpParam)
{
	AcquisitionParameters acqPara = *((AcquisitionParameters*) lpParam);
	CameraPtr pCam = acqPara.pCam;
	cv::Mat* cvImage = acqPara.cvImage;
#else
void* AcquireImages(void* arg)
{
	CameraPtr pCam = *((CameraPtr*)arg);
#endif

    try
    {
		// Use trigger to capture image
		//
		// *** NOTES ***
		
		if (chosenTrigger == SOFTWARE)
		{
			// Execute software trigger
			if (pCam->TriggerSoftware == NULL || pCam->TriggerSoftware.GetAccessMode() != WO)
			{
				cout << "Unable to execute trigger..." << endl;
			}

			pCam->TriggerSoftware.Execute();
		}
		else
		{
			cout << "Use the hardware to trigger image acquisition." << endl;
		}
        // Retrieve next received image
        //
        // *** NOTES ***`
        // Capturing an image houses images on the camera buffer. Trying
        // to capture an image that does not exist will hang the camera.
        //
        // *** LATER ***
        // Once an image from the buffer is saved and/or no longer
        // needed, the image must be released in order to keep the
        // buffer from filling up.
        //
        ImagePtr pResultImage = pCam->GetNextImage();

        // Ensure image completion
        //
        if (pResultImage->IsIncomplete())
        {
            // Retreive and print the image status description
            std::cout << "Image incomplete: "
                    << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                    << "..." << "\n" << endl;
        }
        else
        {
            *cvImage = ConvertToCVmat(pResultImage);
            pResultImage->Release();
            //needs to be converted into BGR(OpenCV uses RGB)
            //cv::cvtColor(cvImage, cvImage, CV_BayerGB2BGR);
			// 使用指针参数，直接对形参赋值
			cv::cvtColor(*cvImage, *cvImage, CV_RGB2HSV);
			return true;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error during acquiring images: " << e.what() << endl;
    }
}
// This function config the camera settings
bool ConfigCamera(CameraPtr pCam)
{
	bool status = true;
	CameraPtr copy = pCam;
    
    std::cout << "\n" << "\n" << "*** IMAGE ACQUISITION ***" << "\n" << endl;
    
    try
    {
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();
		
		// Set acquisition mode to continuous
		if (!IsReadable(pCam->AcquisitionMode) || !IsWritable(pCam->AcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous. Aborting..." << "\n" << endl;
			status = false;
		}

		pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		cout << "Acquisition mode set to continuous..." << endl;
		
		if (pCam->PixelFormat != NULL && pCam->PixelFormat.GetAccessMode() == RW)
		{
			pCam->PixelFormat.SetValue(PixelFormat_RGB8);

			cout << "Pixel format set to " << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..." << endl;
		}
		else
		{
			cout << "Pixel format not available..." << endl;
			status = false;
		}

		// Set maximum width
		//
		// *** NOTES ***
		// Other nodes, such as those corresponding to image width and height, 
		// might have an increment other than 1. In these cases, it can be
		// important to check that the desired value is a multiple of the
		// increment. 
		//
		// This is often the case for width and height nodes. However, because
		// these nodes are being set to their maximums, there is no real reason
		// to check against the increment.
		// The offsetX and offsetY are influenced by height and width
		// So make sure you config height and width, then config offset
		// 偏移量offset受到height 和 width影响，所以先设置宽和高
		if (IsReadable(pCam->Width) && IsWritable(pCam->Width) && pCam->Width.GetInc() != 0 && pCam->Width.GetMax() != 0)
		{
			pCam->Width.SetValue(width); // X 方向宽度的递增量为32， 确保Width是32的整倍数

			cout << "Width set to " << pCam->Width.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Width not available..." << endl;
			status = false;
		}
		if (IsReadable(pCam->Height) && IsWritable(pCam->Height) && pCam->Height.GetInc() != 0 && pCam->Height.GetMax() != 0)
		{
			pCam->Height.SetValue(height);

			cout << "Height set to " << pCam->Height.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Height not available..." << endl;
			status = false;
		}
		if (IsReadable(pCam->OffsetX) && IsWritable(pCam->OffsetX))
		{
			pCam->OffsetX.SetValue(offsetX);

			cout << "Offset X set to " << pCam->OffsetX.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Offset X not available..." << endl;
			status = false;
		}
		//
		if (IsReadable(pCam->OffsetY) && IsWritable(pCam->OffsetY))
		{
			pCam->OffsetY.SetValue(offsetY);

			cout << "Offset Y set to " << pCam->OffsetY.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Offset Y not available..." << endl;
			status = false;
		}
        //process_status = process_status && PrintDeviceInfo(nodeMapTLDevice);
#ifdef _DEBUG
        cout << "\n" << endl << "*** DEBUG ***" << "\n" << endl;
        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, nodeMap, nodeMapTLDevice) != 0)
        {
            return false;
        }
        cout << "\n" << endl << "*** END OF DEBUG ***" << "\n" << endl;
#endif
		
        // The device serial number is retrieved in order to keep cameras from 
        // overwriting one another. Grabbing image IDs could also accomplish
        // this.
        //
		
        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            std::cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
        }
		//
		// Ensure trigger mode off
		//
		// *** NOTES ***
		// The trigger must be disabled in order to configure whether the source
		// is software or hardware.
		//
		if (pCam->TriggerMode == NULL || pCam->TriggerMode.GetAccessMode() != RW)
		{
			cout << "Unable to disable trigger mode. Aborting..." << endl;
			return false;
		}
		if (chosenTrigger == SOFTWARE)
		{
			cout << "Software trigger chosen..." << endl;
		}
		else
		{
			cout << "Hardware trigger chosen..." << endl;
		}
		//
		// Ensure trigger mode off
		//
		// *** NOTES ***
		// The trigger must be disabled in order to configure whether the source
		// is software or hardware.
		//
		pCam->TriggerMode.SetValue(TriggerMode_Off);

		cout << "Trigger mode disabled..." << endl;

		//
		// Select trigger source
		//
		// *** NOTES ***
		// The trigger source must be set to hardware or software while trigger 
		// mode is off.
		//
		if (chosenTrigger == SOFTWARE)
		{
			// Set the trigger source to software
			if (pCam->TriggerSource == NULL || pCam->TriggerSource.GetAccessMode() != RW)
			{
				cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
				return false;
			}

			pCam->TriggerSource.SetValue(TriggerSource_Software);

			cout << "Trigger source set to software..." << endl;
		}
		else
		{
			// Set the trigger source to hardware (using 'Line0')
			if (pCam->TriggerSource == NULL || pCam->TriggerSource.GetAccessMode() != RW)
			{
				cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
				return false;
			}

			pCam->TriggerSource.SetValue(TriggerSource_Line0);

			cout << "Trigger source set to hardware..." << endl;
		}

		//
		// Turn trigger mode on
		//
		// *** LATER ***
		// Once the appropriate trigger source has been set, turn trigger mode 
		// back on in order to retrieve images using the trigger.
		//
		if (pCam->TriggerMode == NULL || pCam->TriggerMode.GetAccessMode() != RW)
		{
			cout << "Unable to disable trigger mode. Aborting..." << endl;
			return false;
		}

		pCam->TriggerMode.SetValue(TriggerMode_On);

		cout << "Trigger mode turned back on..." << "\n" << endl;


		cout << "\n" << endl << "*** CONFIGURING EXPOSURE ***" << "\n" << endl;

		// Turn off automatic exposure mode
		//
		// *** NOTES ***
		// Automatic exposure prevents the manual configuration of exposure 
		// times and needs to be turned off for this example. 
		// *** LATER ***
		// Exposure time can be set automatically or manually as needed. This
		// example turns automatic exposure off to set it manually and back
		// on to return the camera to its default state.
		//
		if (!IsReadable(pCam->ExposureAuto) || !IsWritable(pCam->ExposureAuto))
		{
			cout << "Unable to disable automatic exposure. Aborting..." << "\n" << endl;
			return false;
		}

		pCam->ExposureAuto.SetValue(ExposureAuto_Off);

		cout << "Automatic exposure disabled..." << endl;

		//
		// Set exposure time manually; exposure time recorded in microseconds
		//
		// *** NOTES ***
		// Notice that the node is checked for availability and writability
		// prior to the setting of the node. In QuickSpin, availability is
		// ensured by checking for null while writability is ensured by checking
		// the access mode. 
		//
		// Further, it is ensured that the desired exposure time does not exceed 
		// the maximum. Exposure time is counted in microseconds - this can be 
		// found out either by retrieving the unit with the GetUnit() method or 
		// by checking SpinView.
		// 
		if (!IsReadable(pCam->ExposureTime) || !IsWritable(pCam->ExposureTime))
		{
			cout << "Unable to set exposure time. Aborting..." << "\n" << endl;
			return false;
		}

		// Ensure desired exposure time does not exceed the maximum
		const double exposureTimeMax = pCam->ExposureTime.GetMax();
		double exposureTimeToSet = 17000.0;

		if (exposureTimeToSet > exposureTimeMax)
		{
			exposureTimeToSet = exposureTimeMax;
		}

		pCam->ExposureTime.SetValue(exposureTimeToSet);

		cout << std::fixed << "Shutter time set to " << exposureTimeToSet << " us..." << "\n" << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Spinnaker Error during config Camera: " << e.what() << endl;
        status = false;
    }
    catch (cv::Exception &e)
    {
        std::cout << "CV Error during config Camera: " << e.what() << endl;
        status = false;
    }
    return status;
}
// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap)
{
    bool process_status = true;
    
    std::cout << "\n" << "*** DEVICE INFORMATION ***" << "\n" << endl;

    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                std::cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                std::cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                std::cout << endl;
            }
        }
        else
        {
            std::cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error: during print Device information" << e.what() << endl;
        process_status = false;
    }
    
    return process_status;
}

// This function returns the camera to a normal state by re-enabling automatic
// exposure.
int ResetExposure(CameraPtr pCam)
{
	int result = 0;

	try
	{
		// 
		// Turn automatic exposure back on
		//
		// *** NOTES ***
		// Automatic exposure is turned on in order to return the camera to its 
		// default state.
		//
		if (!IsReadable(pCam->ExposureAuto) || !IsWritable(pCam->ExposureAuto))
		{
			cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
			return -1;
		}

		pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);

		cout << "Automatic exposure enabled..." << endl << endl;
	}
	catch (Spinnaker::Exception& e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}