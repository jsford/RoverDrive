#include "pr_utils/pr_pitcam.h"
#include "pr_utils/pr_log.h"
#include "pr_utils/pr_math.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <fmt/format.h>
#include <cmath>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace pr {

const float   PITCAM_WHITE_BALANCE_BLUE_RATIO =     1.83f;
const float   PITCAM_WHITE_BALANCE_RED_RATIO  =     1.40f;
const float   PITCAM_GAIN                     =      9.0f;
const float   PITCAM_GAMMA                    =  1.0/1.8f;
const int64_t PITCAM_DEVICE_LINK_THROUGHPUT_LIMIT_MBPS = 10000000;

PitCamera::PitCamera() {

    pSystem = System::GetInstance();
    camList = pSystem->GetCameras();

    if( camList.GetSize() < 1 ) {
        camList.Clear();
        pSystem->ReleaseInstance();
        throw std::runtime_error("Failed to find a Spinnaker camera.");
    }

    pCam = camList.GetByIndex(0);

    // Initialize the camera.
    pCam->Init();

    // Set device link throughput.
    set_device_link_throughput(PITCAM_DEVICE_LINK_THROUGHPUT_LIMIT_MBPS);

    // Tell the camera to use jumbo packets.
    use_jumbo_packets();

    // Set the camera gain to something that is hopefully reasonable.
    set_gain(PITCAM_GAIN);

    // Use manual white balance.
    set_white_balance();

    // Set the initial exposure to the autoexposure value.
    set_exposure(get_autoexposure());

    // Set gamma correction.
    set_gamma(PITCAM_GAMMA);
}

PitCamera::~PitCamera() {
    // Deinitialize the camera.
    pCam->DeInit();

    pCam = nullptr;
    camList.Clear();
    pSystem->ReleaseInstance();
}

void PitCamera::use_jumbo_packets() {
    pCam->GevSCPSPacketSize = 9000;
}

void PitCamera::set_gamma(float g) {
  // Enable gamma correction.
  CBooleanPtr ptrGammaEnable = pCam->GetNodeMap().GetNode("GammaEnable");
  ptrGammaEnable->SetValue(true);
  // Set the value of gamma.
  CFloatPtr ptrGamma = pCam->GetNodeMap().GetNode("Gamma");
  ptrGamma->SetValue(PITCAM_GAMMA);
}

void PitCamera::set_device_link_throughput(int MBps) {
    CIntegerPtr ptrDeviceLinkThroughputLimit = pCam->GetNodeMap().GetNode("DeviceLinkThroughputLimit");
    if (!IsAvailable(ptrDeviceLinkThroughputLimit) || !IsWritable(ptrDeviceLinkThroughputLimit)) {
        throw std::runtime_error("Unable to set device link throughput limit (node retrieval). Aborting...");
    }
    ptrDeviceLinkThroughputLimit->SetValue(MBps);
}

int PitCamera::get_autoexposure() {
    // Save current exposure so we can restore it later.
    int original_exposure = get_exposure();

    // Enable auto exposure and capture images to allow it to converge.
    use_autoexposure();

    // Iterate 5-20 times or until the exposure converges within 5%.
    // NOTE(Jordan): I have to force at least five
    // image captures to get the camera to reliably change exposures.
    int last_exposure = -1;
    fmt::print("INIT EXPOSURE: {}\n", get_exposure());
    for(int i=0; i<10; ++i) {
        capture(0);
        int exposure = get_exposure();
        fmt::print("EXPOSURE: {}\n", exposure);
        if(last_exposure > 0 && std::abs(exposure-last_exposure) < 0.05*exposure) {
          break;
        }
        last_exposure = exposure;
    }
    // Save the calculated auto exposure.
    int autoexposure = get_exposure();

    // Reset to original exposure.
    use_manual_exposure();
    set_exposure(original_exposure);

    // Return the optimal exposure.
    return autoexposure;
}

int PitCamera::get_max_exposure() {
    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    return ptrExposureTime->GetMax();
}

int PitCamera::get_min_exposure() {
    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    return ptrExposureTime->GetMin();
}

bool PitCamera::get_autoexposure_status() {
  // If the camera is set to continuous autoexposure, return true. Else, return false.
  CEnumerationPtr ptrExposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
  if (!IsAvailable(ptrExposureAuto)) {
      throw std::runtime_error("Unable to retrieve ExposureAuto node.");
  }

  CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
  if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous)) {
      throw std::runtime_error("Unable to retrieve ExposureAuto 'Continuous' entry node.");
  }
  return (ptrExposureAuto->GetIntValue() == ptrExposureAutoContinuous->GetValue());
}

int PitCamera::get_exposure() {
  // The camera will not tell you the exposure it is using if it is in autoexposure mode.
  // Instead, you must switch to manual mode, query the current exposure, then return to autoexposure mode.

  // Is autoexposure on?
  bool autoexposure_enabled = get_autoexposure_status();

  // If so, turn it off.
  if(autoexposure_enabled) {
    use_manual_exposure();
  }

  // What is the current exposure time?
  CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
  if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
      throw std::runtime_error("Unable to retrieve ExposureTime node.");
  }
  int exposure = ptrExposureTime->GetValue();

  // Restore autoexposure if necessary.
  if(autoexposure_enabled) {
    use_autoexposure();
  }
  return exposure;
}

void PitCamera::set_exposure(int exposure_us) {
    use_manual_exposure();

    int exp_min = get_min_exposure();
    int exp_max = get_max_exposure();
    exposure_us = pr::clamp(exposure_us, exp_min, exp_max);

    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    ptrExposureTime->SetValue(exposure_us);
}

void PitCamera::set_gain(double gain) {
  CEnumerationPtr gainAuto = pCam->GetNodeMap().GetNode("GainAuto");
  gainAuto->SetIntValue(gainAuto->GetEntryByName("Off")->GetValue());

  CFloatPtr gainValue = pCam->GetNodeMap().GetNode("Gain");
  gainValue->SetValue(gain);
}

void PitCamera::set_white_balance() {
  // Disable Automatic White Balance Adjustment.
  CEnumerationPtr ptrBalanceWhiteAuto = pCam->GetNodeMap().GetNode("BalanceWhiteAuto");
  CEnumEntryPtr ptrBalanceWhiteAutoOff = ptrBalanceWhiteAuto->GetEntryByName("Off");
  ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoOff->GetValue());

  // Manually set the G/R White Balance Ratio.
  {
    CEnumerationPtr ptrBalanceRatioSelector = pCam->GetNodeMap().GetNode("BalanceRatioSelector");
    CEnumEntryPtr ptrBalanceRatioSelectorRed = ptrBalanceRatioSelector->GetEntryByName("Red");
    ptrBalanceRatioSelector->SetIntValue(ptrBalanceRatioSelectorRed->GetValue());
    CFloatPtr ptrBalanceRatio = pCam->GetNodeMap().GetNode("BalanceRatio");
    ptrBalanceRatio->SetValue(PITCAM_WHITE_BALANCE_RED_RATIO);
  }

  // Manually set the G/B White Balance Ratio.
  {
    CEnumerationPtr ptrBalanceRatioSelector = pCam->GetNodeMap().GetNode("BalanceRatioSelector");
    CEnumEntryPtr ptrBalanceRatioSelectorRed = ptrBalanceRatioSelector->GetEntryByName("Blue");
    ptrBalanceRatioSelector->SetIntValue(ptrBalanceRatioSelectorRed->GetValue());
    CFloatPtr ptrBalanceRatio = pCam->GetNodeMap().GetNode("BalanceRatio");
    ptrBalanceRatio->SetValue(PITCAM_WHITE_BALANCE_BLUE_RATIO);
  }
}

PitCamera::ImageRGB
PitCamera::capture(int exposure_us) {
    if (exposure_us > 0) {
        set_exposure(exposure_us);
    }

    // Retrieve the GenICam nodemap.
    auto &nodeMap = pCam->GetNodeMap();

    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        throw std::runtime_error("Unable to set acquisition mode to 'continuous'. Aborting...");
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        throw std::runtime_error("Unable to set acquisition mode to 'continuous'. Aborting...");
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    int retry;
    const int num_retries = 10;
    for(retry=0; retry < num_retries; ++retry) {
      if(retry > 0) {
        fmt::print("Image Capture Retry #{}\n", retry);
      }
 
      pCam->BeginAcquisition();
        // Retrieve next received image
        ImagePtr pResultImage = pCam->GetNextImage();

        // Ensure image completion
        if (pResultImage->IsIncomplete()) {
            pResultImage->Release();
            pCam->EndAcquisition();
        } else {
            // Convert to RGB8 (if the image is not RGB8 already)
            ImagePtr pImageRGB = pResultImage->Convert(PixelFormat_RGB8);

            // Convert Spinnaker::Image to ImageRGB
            const size_t width = pImageRGB->GetWidth();
            const size_t height = pImageRGB->GetHeight();
            const size_t channels = pImageRGB->GetNumChannels();
            const uint8_t* img_data = static_cast<const uint8_t*>(pImageRGB->GetData());

            if( pImageRGB->GetXPadding() != 0 || pImageRGB->GetYPadding() != 0 ) {
                std::runtime_error("PitCam image has unexpected nonzero padding.");
            }

            // Copy the image data into our own image format.
            PitCamera::ImageRGB image;
            image.cols = width;
            image.rows = height;
            image.data.resize(width*height*channels);

            std::memcpy(&image.data[0], img_data, channels*width*height);

            pResultImage->Release();
            pCam->EndAcquisition();

            return image;
        }
    }

    // If we used all our retries, throw an exception.
    auto msg = fmt::format("PitCam failed to capture a complete image after {} attempts.", num_retries);
    throw std::runtime_error(msg);
}

void PitCamera::use_autoexposure() {
  // Enable continuous autoexposure.
  CEnumerationPtr ptrExposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
  if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto)) {
      throw std::runtime_error("Unable to retrieve ExposureAuto node.");
  }

  CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
  if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous)) {
      throw std::runtime_error("Unable to retrieve ExposureAuto 'Continuous' entry node.");
  }
  ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());

  // Set the autoexposure range to the maximum possible.
  CFloatPtr ptrExposureUpperLimit = pCam->GetNodeMap().GetNode("AutoExposureExposureTimeUpperLimit");
  ptrExposureUpperLimit->SetValue(get_max_exposure());
  CFloatPtr ptrExposureLowerLimit = pCam->GetNodeMap().GetNode("AutoExposureExposureTimeLowerLimit");
  ptrExposureLowerLimit->SetValue(get_min_exposure());
}

void PitCamera::use_manual_exposure() {
    CEnumerationPtr ptrExposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto node.");
    }

    CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto 'Off' entry node.");
    }
    ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
}

} // namespace pr

