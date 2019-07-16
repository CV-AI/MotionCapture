//=============================================================================
// Copyright © 2018 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
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
//=============================================================================*/

/**
 *  @example AcquisitionMultipleCamera.cpp
 *
 *  @brief AcquisitionMultipleCamera.cpp shows how to capture images from
 *  multiple cameras simultaneously. It relies on information provided in the 
 *  Enumeration, Acquisition, and NodeMapInfo examples.
 *
 *  This example reads similarly to the Acquisition example, except that loops
 *  and vectors are used to allow for simultaneous acquisitions.
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream> 

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
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
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
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

// This function acquires and saves 10 images from each device.  
int AcquireImages(CameraList camList)
{
    int result = 0;
    CameraPtr pCam = NULL;

    cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    try
    {
        //
        // Prepare each camera to acquire images
        // 
        // *** NOTES ***
        // For pseudo-simultaneous streaming, each camera is prepared as if it 
        // were just one, but in a loop. Notice that cameras are selected with 
        // an index. We demonstrate pseduo-simultaneous streaming because true 
        // simultaneous streaming would require multiple process or threads,
        // which is too complex for an example. 
        // 
        // Serial numbers are the only persistent objects we gather in this
        // example, which is why a vector is created.
        //
        vector<gcstring> strSerialNumbers(camList.GetSize());

        for (unsigned int i = 0; i < camList.GetSize(); i++)
        {
            // Select camera
            pCam = camList.GetByIndex(i);

            // Set acquisition mode to continuous
            CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
            if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
            {
                cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting..." << endl << endl;
                return -1;
            }

            CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
            if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
            {
                cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting..." << endl << endl;
                return -1;
            }

            int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

            ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

            cout << "Camera " << i << " acquisition mode set to continuous..." << endl;

#ifdef _DEBUG
            cout << endl << endl << "*** DEBUG ***" << endl << endl;
            // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
            if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0)
            {
                return -1;
            }

            cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif
            // Begin acquiring images
            pCam->BeginAcquisition();

            cout << "Camera " << i << " started acquiring images..." << endl;

            // Retrieve device serial number for filename
            strSerialNumbers[i] = "";

            CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

            if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
            {
                strSerialNumbers[i] = ptrStringSerial->GetValue();
                cout << "Camera " << i << " serial number set to " << strSerialNumbers[i] << "..." << endl;
            }

            cout << endl;
        }

        //
        // Retrieve, convert, and save images for each camera
        //
        // *** NOTES ***
        // In order to work with simultaneous camera streams, nested loops are
        // needed. It is important that the inner loop be the one iterating
        // through the cameras; otherwise, all images will be grabbed from a
        // single camera before grabbing any images from another.
        //
        const unsigned int k_numImages = 10;

        for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
        {
            for (unsigned int i = 0; i < camList.GetSize(); i++)
            {
                try
                {
                    // Select camera
                    pCam = camList.GetByIndex(i);

                    // Retrieve next received image and ensure image completion
                    ImagePtr pResultImage = pCam->GetNextImage();

                    if (pResultImage->IsIncomplete())
                    {
                        cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
                    }
                    else
                    {
                        // Print image information
                        cout << "Camera " << i << " grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;

                        // Convert image to mono 8
                        ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

                        // Create a unique filename
                        ostringstream filename;

                        filename << "AcquisitionMultipleCamera-";
                        if (strSerialNumbers[i] != "")
                        {
                            filename << strSerialNumbers[i].c_str();
                        }
                        else
                        {
                            filename << i;
                        }
                        filename << "-" << imageCnt << ".jpg";

                        // Save image
                        convertedImage->Save(filename.str().c_str());

                        cout << "Image saved at " << filename.str() << endl;
                    }

                    // Release image
                    pResultImage->Release();

                    cout << endl;
                }
                catch (Spinnaker::Exception &e)
                {
                    cout << "Error: " << e.what() << endl;
                    result = -1;
                }
            }
        }

        //
        // End acquisition for each camera
        //
        // *** NOTES ***
        // Notice that what is usually a one-step process is now two steps
        // because of the additional step of selecting the camera. It is worth
        // repeating that camera selection needs to be done once per loop.
        //
        // It is possible to interact with cameras through the camera list with
        // GetByIndex(); this is an alternative to retrieving cameras as 
        // CameraPtr objects that can be quick and easy for small tasks.
        //
        for (unsigned int i = 0; i < camList.GetSize(); i++)
        {
            // End acquisition
            camList.GetByIndex(i)->EndAcquisition();
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap, unsigned int camNum)
{
    int result = 0;

    cout << "Printing device information for camera " << camNum << "..." << endl << endl;

    FeatureList_t features;
    CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category))
    {
        category->GetFeatures(features);

        FeatureList_t::const_iterator it;
        for (it = features.begin(); it != features.end(); ++it)
        {
            CNodePtr pfeatureNode = *it;
            cout << pfeatureNode->GetName() << " : ";
            CValuePtr pValue = (CValuePtr)pfeatureNode;
            cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
            cout << endl;
        }
    }
    else
    {
        cout << "Device control information not available." << endl;
    }
    cout << endl;

    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int RunMultipleCameras(CameraList camList)
{
    int result = 0;
    CameraPtr pCam = NULL;

    try
    {
        //
        // Retrieve transport layer nodemaps and print device information for 
        // each camera
        //
        // *** NOTES ***
        // This example retrieves information from the transport layer nodemap 
        // twice: once to print device information and once to grab the device 
        // serial number. Rather than caching the nodemap, each nodemap is 
        // retrieved both times as needed.
        //
        cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

        for (unsigned int i = 0; i < camList.GetSize(); i++)
        {
            // Select camera
            pCam = camList.GetByIndex(i);

            // Retrieve TL device nodemap
            INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

            // Print device information
            result = PrintDeviceInfo(nodeMapTLDevice, i);
        }

        //
        // Initialize each camera
        // 
        // *** NOTES ***
        // You may notice that the steps in this function have more loops with
        // less steps per loop; this contrasts the AcquireImages() function 
        // which has less loops but more steps per loop. This is done for
        // demonstrative purposes as both work equally well.
        //
        // *** LATER ***
        // Each camera needs to be deinitialized once all images have been 
        // acquired.
        //
        for (unsigned int i = 0; i < camList.GetSize(); i++)
        {
            // Select camera
            pCam = camList.GetByIndex(i);

            // Initialize camera
            pCam->Init();
        }
        
        // Acquire images on all cameras
        result = result | AcquireImages(camList);

        // 
        // Deinitialize each camera
        //
        // *** NOTES ***
        // Again, each camera must be deinitialized separately by first
        // selecting the camera and then deinitializing it.
        //
        for (unsigned int i = 0; i < camList.GetSize(); i++)
        {
            // Select camera
            pCam = camList.GetByIndex(i);

            // Deinitialize camera
            pCam->DeInit();
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
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
        cout << "Failed to create file in current folder.  Please check "
            "permissions."
            << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    int result = 0;

    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    // Run example on all cameras
    cout << endl << "Running example for all cameras..." << endl;

    result = RunMultipleCameras(camList);

    cout << "Example complete..." << endl << endl;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return result;
}
