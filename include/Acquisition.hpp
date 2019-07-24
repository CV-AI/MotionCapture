//=============================================================================
// Copyright © 2018 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

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
    return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, spinImage->GetData(), spinImage->GetStride());
}
// This function acquires images from a device.
cv::Mat AcquireImages(CameraPtr  pCam)
{
    
    cv::Mat cvImage;
    // Retrieve, convert, and show images

    try
    {

        // Retrieve next received image
        //
        // *** NOTES ***
        // Capturing an image houses images on the camera buffer. Trying
        // to capture an image that does not exist will hang the camera.
        //
        // *** LATER ***
        // Once an image from the buffer is saved and/or no longer
        // needed, the image must be released in order to keep the
        // buffer from filling up.
        //
        ImagePtr pResultImage = pCam->GetNextImage();

         //
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
            // original image is in RGB format, needs to be converted into BGR(OpenCV uses BGR)
            cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
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

        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << "\n" << endl;
            return -1;
        }
        
        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            std::cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << "\n" << endl;
            return -1;
        }
        
        // Retrieve integer value from entry node
        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        
        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        
        std::cout << "Acquisition mode set to continuous..." << endl;

		// Turn off auto exposure
		pCam->ExposureAuto.SetValue(Spinnaker::ExposureAutoEnums::ExposureAuto_Off);
		//Set exposure mode to "Timed"
		pCam->ExposureMode.SetValue(Spinnaker::ExposureModeEnums::ExposureMode_Timed);
		//Set absolute value of shutter exposure time to microseconds
		pCam->ExposureTime.SetValue(10000);
		/*pCam->AasRoiEnable.SetValue(Spinnaker::AutoAlgorithmSelectorEnums::AutoAlgorithmSelector_Ae);
		pCam->AasRoiHeight.SetValue(1024);
		pCam->AasRoiWidth.SetValue(1024);*/
        if (pCam->PixelFormat != NULL && pCam->PixelFormat.GetAccessMode() == RW)
		{
			pCam->PixelFormat.SetValue(PixelFormat_RGB8);

			cout << "Pixel format set to " << pCam->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..." << endl;
		}
		else
		{               
			cout << "Pixel format not available..." << endl;
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
			pCam->OffsetX.SetValue(pCam->OffsetX.GetMin());

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
			pCam->OffsetY.SetValue(pCam->OffsetY.GetMin());

			cout << "Offset Y set to " << pCam->OffsetY.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Offset Y not available..." << endl;
			process_status = false;
		}

		//
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
		//
		if (IsReadable(pCam->Width) && IsWritable(pCam->Width) && pCam->Width.GetInc() != 0 && pCam->Width.GetMax() != 0)
		{
			pCam->Width.SetValue(pCam->Width.GetMax());

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
			pCam->Height.SetValue(pCam->Height.GetMax());

			cout << "Height set to " << pCam->Height.GetValue() << "..." << endl;
		}
		else
		{
			cout << "Height not available..." << endl;
			process_status = false;
		}
        // BeginAcquisition after camera pixelFormat was set and before printDeviceInfo
        pCam->BeginAcquisition();
        process_status = process_status && PrintDeviceInfo(nodeMapTLDevice);
#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;
        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }
        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        //
        // Begin acquiring images
        //
        // *** NOTES ***
        // What happens when the camera begins acquiring images depends on the
        // acquisition mode. Single frame captures only a single image, multi 
        // frame catures a set number of images, and continuous captures a 
        // continuous stream of images. Because the example calls for the 
        // retrieval of 10 images, continuous mode has been set.
        // 
        // *** LATER ***
        // Image acquisition must be ended when no more images are needed.
        
        //
        // Retrieve device serial number for filename
        //
        // *** NOTES ***
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