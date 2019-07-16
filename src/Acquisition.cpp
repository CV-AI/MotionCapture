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
    cv::Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, spinImage->GetData(), spinImage->GetStride());
    return cvimg;
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
            // Print image information; height and width recorded in pixels
            cvImage = ConvertToCVmat(pResultImage);
            // original image is in RGB format, needs to be converted into BGR(OpenCV uses BGR)
            cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
        }

        // Release image
        //
        // *** NOTES ***
        // Images retrieved directly from the camera (i.e. non-converted
        // images) need to be released in order to keep from filling the
        // buffer.
        //
        pResultImage->Release();
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error during acquiring images: " << e.what() << endl;
    }

    //
    // End acquisition
    //
    // *** NOTES ***
    // Ending acquisition appropriately helps ensure that devices clean up
    // properly and do not need to be power-cycled to maintain integrity.
    //
    return cvImage;
}
// This function config the camera settings
int ConfigCamera(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
    int result = 0;
    
    std::cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
    
    try
    {
        //
        // Set acquisition mode to continuous
        //
        // *** NOTES ***
        // Because the example acquires and saves 10 images, setting acquisition 
        // mode to continuous lets the example finish. If set to single frame
        // or multiframe (at a lower number of images), the example would just
        // hang. This would happen because the example has been written to
        // acquire 10 images while the camera would have been programmed to 
        // retrieve less than that.
        // 
        // Setting the value of an enumeration node is slightly more complicated
        // than other node types. Two nodes must be retrieved: first, the 
        // enumeration node is retrieved from the nodemap; and second, the entry
        // node is retrieved from the enumeration node. The integer value of the
        // entry node is then set as the new value of the enumeration node.
        //
        // Notice that both the enumeration and the entry nodes are checked for
        // availability and readability/writability. Enumeration nodes are
        // generally readable and writable whereas their entry nodes are only
        // ever readable.
        // 
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            return -1;
        }
        
        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            std::cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
            return -1;
        }
        
        // Retrieve integer value from entry node
        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        
        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        
        std::cout << "Acquisition mode set to continuous..." << endl;

        // Set pixel format to RGB8
        CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
        if (!IsAvailable(ptrPixelFormat) || !IsWritable(ptrPixelFormat))
        {
            std::cout<< "Unable to set Pixel Format to RGB8(enum retrieval). Aborting"<<endl;
            return -1;
        }
        CEnumEntryPtr ptrPixelFormat_RGB8 = ptrPixelFormat->GetEntryByName("RGB8");
        if ((!IsAvailable(ptrPixelFormat_RGB8)) || !IsReadable(ptrPixelFormat_RGB8))
        {
            std::cout<< "Unable to set Pixel Format to RGB8(entry retrieval). Aborting"<<endl;
            return -1;
        }
        int64_t pixelFormatRGB8 = ptrPixelFormat_RGB8->GetValue();
        ptrPixelFormat->SetIntValue(pixelFormatRGB8);

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
        // pCam->BeginAcquisition();

        std::cout << "Configing Camera..." << endl;
        
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
        std::cout << endl;

    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error during config Camera: " << e.what() << endl;
        result = -1;
    }
    return result;
}
// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap)
{
    int result = 0;
    
    std::cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

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
        result = -1;
    }
    
    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
    int result = 0;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();
        
        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();
        // Acquire images
        ConfigCamera(pCam, nodeMap, nodeMapTLDevice);
        pCam->BeginAcquisition();
        cv::Mat image;
        cv::namedWindow("Camera", 0);
        while(true)
        {
            image = AcquireImages(pCam);
            cv::imshow("Camera", image);
            cv::waitKey(1);
            int key = cv::waitKey(1);
            if ( key == 27) // press "Esc" to stop
            {break;}
        } 
        // Deinitialize camera
        pCam->EndAcquisition();
        pCam->DeInit();
    }
    catch (Spinnaker::Exception &e)
    {
        std::cout << "Error during RunSingleCamera: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE *tempFile = fopen("test.txt", "w+");
    if (tempFile == NULL)
    {
        std::cout << "Failed to create file in current folder.  Please check "
            "permissions."
            << endl;
        std::cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    int result = 0;
    
    // Print application build information
    std::cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
    
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

    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to NULL.
    // This keeps releasing the system from throwing an exception.
    //
    CameraPtr pCam = NULL;

    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);

        std::cout << endl << "Running example for camera " << i << "..." << endl;
        
        // Run example
        result = result | RunSingleCamera(pCam);
        
        std::cout << "Camera " << i << " example complete..." << endl << endl;
    }

    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = NULL;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    std::cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();
    return result;
}
