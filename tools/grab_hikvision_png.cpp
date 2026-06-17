#include "MvCameraControl.h"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool CheckMvRet(const std::string &call, int ret) {
  if (ret == MV_OK) {
    return true;
  }
  std::cerr << call << " failed: 0x" << std::hex << ret << std::dec << " (" << ret << ")\n";
  return false;
}

std::string DeviceSerial(const MV_CC_DEVICE_INFO *info) {
  if (info == nullptr) {
    return "";
  }
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  return "";
}

std::string DeviceModel(const MV_CC_DEVICE_INFO *info) {
  if (info == nullptr) {
    return "";
  }
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chModelName);
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chModelName);
  }
  return "";
}

}  // namespace

int main(int argc, char **argv) {
  const std::string output_path = argc > 1 ? argv[1] : "/home/vision/FAST-Calib/calib_data/current_static_pair/image.png";
  const std::string expected_serial = argc > 2 ? argv[2] : "DA3217436";
  const float exposure_us = argc > 3 ? std::stof(argv[3]) : 30000.0f;
  const float gain = argc > 4 ? std::stof(argv[4]) : 15.0f;

  MV_CC_DEVICE_INFO_LIST devices;
  std::memset(&devices, 0, sizeof(devices));
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
  if (!CheckMvRet("MV_CC_EnumDevices", ret)) {
    return 2;
  }

  int selected = -1;
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    const MV_CC_DEVICE_INFO *info = devices.pDeviceInfo[i];
    std::cout << "device[" << i << "] model=" << DeviceModel(info)
              << " serial=" << DeviceSerial(info) << "\n";
    if (DeviceSerial(info) == expected_serial) {
      selected = static_cast<int>(i);
    }
  }
  if (selected < 0) {
    std::cerr << "serial " << expected_serial << " not found\n";
    return 3;
  }

  void *handle = nullptr;
  ret = MV_CC_CreateHandle(&handle, devices.pDeviceInfo[selected]);
  if (!CheckMvRet("MV_CC_CreateHandle", ret)) {
    return 4;
  }

  ret = MV_CC_OpenDevice(handle);
  if (!CheckMvRet("MV_CC_OpenDevice", ret)) {
    MV_CC_DestroyHandle(handle);
    return 5;
  }

  MV_CC_SetEnumValueByString(handle, "AcquisitionMode", "Continuous");
  MV_CC_SetEnumValueByString(handle, "TriggerMode", "Off");
  MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8);
  MV_CC_SetExposureAutoMode(handle, 0);
  MV_CC_SetExposureTime(handle, exposure_us);
  MV_CC_SetEnumValue(handle, "GainAuto", 0);
  MV_CC_SetGain(handle, gain);

  ret = MV_CC_StartGrabbing(handle);
  if (!CheckMvRet("MV_CC_StartGrabbing", ret)) {
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 6;
  }

  MVCC_INTVALUE_EX payload;
  std::memset(&payload, 0, sizeof(payload));
  ret = MV_CC_GetIntValueEx(handle, "PayloadSize", &payload);
  if (!CheckMvRet("MV_CC_GetIntValueEx(PayloadSize)", ret)) {
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 7;
  }

  std::vector<unsigned char> buffer(static_cast<size_t>(payload.nCurValue));
  MV_FRAME_OUT_INFO_EX info;
  std::memset(&info, 0, sizeof(info));
  ret = MV_CC_GetOneFrameTimeout(handle, buffer.data(), static_cast<unsigned int>(buffer.size()), &info, 3000);
  if (!CheckMvRet("MV_CC_GetOneFrameTimeout", ret)) {
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 8;
  }

  cv::Mat bayer(static_cast<int>(info.nHeight), static_cast<int>(info.nWidth), CV_8UC1, buffer.data());
  cv::Mat bgr;
  cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
  if (!cv::imwrite(output_path, bgr)) {
    std::cerr << "failed to write " << output_path << "\n";
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 9;
  }

  std::cout << "saved " << output_path << " width=" << info.nWidth << " height=" << info.nHeight
            << " exposure_us=" << exposure_us << " gain=" << gain << "\n";

  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);
  return 0;
}
