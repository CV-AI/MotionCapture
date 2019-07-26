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
const int64_t height = 1440;
const int64_t width = 1440;
#ifdef _DEBUG
// Disables heartbeat on GEV cameras(GigE Vison) so debugging does not incur timeout errors
int DisableHeartbeat(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;
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
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if ( !IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat) )
            {
                cout << "Unable to disable heartbeat on camera. Aborting..." << endl << endl;
                return -1;
            }
            ptrDeviceHeartbeat->SetValue(true);
            cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
            cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif
cv::Mat ConvertToCVmat(ImagePtr spinImage);
cv::Mat AcquireImages(CameraPtr  pCam);
int ConfigCamera(CameraPtr pCam);
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
    return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, spinImage->GetData(), spinImage->GetStride());
}
// This function acquires images from a device.
cv::Mat AcquireImages(CameraPtr  pCam)
{
    
    cv::Mat cvImage;
    // Retrieve, convert, and show images

    try
    {
		// Use trigger to capture image
		//
		// *** NOTES ***
		// The software trigger only feigns being executed by the Enter key;
		// what might not be immediately apparent is that there is no 
		// continuous stream of images being captured; in other examples that 
		// acquire images, the camera captures a continuous stream of images. 
		// When an image is then retrieved, it is plucked from the stream; 
		// there are many more images captured than retrieved. However, while 
		// trigger mode is activated, there is only a single image captured at 
		// the time that the trigger is activated. 
		//
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
                    << "..." << endl << endl;
        }
        else
        {
            cvImage = ConvertToCVmat(pResultImage);
            pResultImage->Release();
            //needs to be converted into BGR(OpenCV uses RGB)
            cv::cvtColor(cvImage, cvImage, CV_BayerGB2RGB);
			cv::cvtColor(cvImage, cvImage, CV_BGR2HSV);
        }
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error during acquiring images: " << e.what() << endl;
    }
    return cvImage;
}
// This function config the camera settings
int ConfigCamera(CameraPtr pCam)
{
    bool process_status = true;
    
    std::cout << "\n" << "\n" << "*** IMAGE ACQUISITION ***" << "\n" << endl;
    
    try
    {
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();

		// Set acquisition mode to continuous
		if (!IsReadable(pCam->AcquisitionMode) || !IsWritable(pCam->AcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous. Aborting..." << endl << endl;
			return -1;
		}

		pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		cout << "Acquisition mode set to continuous..." << endl;

		//// Turn off auto exposure
		//pCam->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
		////Set exposure mode to "Timed"
		//pCam->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);
		////Set absolute value of shutter exposure time to microseconds
		//pCam->ExposureTime.SetValue(10000);
		
        if (pCam->PixelFormat != NULL && pCam->PixelFormat.GetAccessMode() == RW)
		{
			pCam->PixelFormat.SetValue(PixelFormat_BayerGB8);

			cout << "Pixel format set to " << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..." << endl;
		}
		else
		{               
			cout << "Pixel format not available..." << endl;
			process_status = false;
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
			process_status = false;
		}

		//
		// Set maximum height
		//
		// *** NOTES ***
		// A maximum is retrieved with the method GetMax(). A node's minimum and
		// maximum should always be a multiple of its increment.
		//
		if (IsReadable(pCam->Height) && IsWritable(pCam->Height) && pCam->Height.GetInc() != 0 && pCam->Height.GetMax() != 0)
		{
			pCam->Height.SetValue(height);

			cout << "Height set to " << pCam->Height.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Height not available..." << endl;
			process_status = false;
		}
		// Apply minimum to offset X
		//
		// *** NOTES ***
		// Numeric nodes have both a minimum and maximum. A minimum is retrieved
		// with the method GetMin(). Sometimes it can be important to check 
		// minimums to ensure that your desired value is within range.
		//
		if (IsReadable(pCam->OffsetX) && IsWritable(pCam->OffsetX))
		{
			pCam->OffsetX.SetValue(offsetX);

			cout << "Offset X set to " << pCam->OffsetX.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Offset X not available..." << endl;
			process_status = false;
		}

		//
		// Apply minimum to offset Y
		// 
		// *** NOTES ***
		// It is often desirable to check the increment as well. The increment
		// is a number of which a desired value must be a multiple. Certain
		// nodes, such as those corresponding to offsets X and Y, have an
		// increment of 1, which basically means that any value within range
		// is appropriate. The increment is retrieved with the method GetInc().
		//
		if (IsReadable(pCam->OffsetY) && IsWritable(pCam->OffsetY))
		{
			pCam->OffsetY.SetValue(offsetY);

			cout << "Offset Y set to " << pCam->OffsetY.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Offset Y not available..." << endl;
			process_status = false;
		}

		//
		
        //process_status = process_status && PrintDeviceInfo(nodeMapTLDevice);
#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;
        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }
        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
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
		if (pCam->TriggerMode == NULL || pCam->TriggerMode.GetAccessMode() != RW)
		{
			cout << "Unable to disable trigger mode. Aborting..." << endl;
			return -1;
		}

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
				return -1;
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
				return -1;
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
			return -1;
		}

		pCam->TriggerMode.SetValue(TriggerMode_On);

		cout << "Trigger mode turned back on..." << endl << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Spinnaker Error during config Camera: " << e.what() << endl;
        process_status = false;
    }
    catch (cv::Exception &e)
    {
        std::cout << "CV Error during config Camera: " << e.what() << endl;
        process_status = false;
    }
    return process_status;
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