/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AVT_VIMBA_API_H
#define AVT_VIMBA_API_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>

#include <string>
#include <map>

using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;
using AVT::VmbAPI::VimbaSystem;

namespace avt_vimba_camera
{
class AvtVimbaApi
{
public:
  AvtVimbaApi(const rclcpp::Logger& logger) : vs(VimbaSystem::GetInstance()), logger_(logger)
  {
  }

  void start()
  {
    VmbErrorType err = vs.Startup();
    if (VmbErrorSuccess == err)
    {
      RCLCPP_INFO_STREAM(logger_, "[Vimba System]: AVT Vimba System initialized successfully");
      listAvailableCameras();
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "[Vimba System]: Could not start Vimba system: " << errorCodeToMessage(err));
    }
  }

  /** Translates Vimba error codes to readable error messages
   *
   * @param error Vimba error type
   * @return readable string error
   *
   **/
  std::string errorCodeToMessage(VmbErrorType error)
  {
    std::map<VmbErrorType, std::string> error_msg;
    error_msg[VmbErrorSuccess] = "Success.";
    error_msg[VmbErrorApiNotStarted] = "API not started.";
    error_msg[VmbErrorNotFound] = "Not found.";
    error_msg[VmbErrorBadHandle] = "Invalid handle ";
    error_msg[VmbErrorDeviceNotOpen] = "Device not open.";
    error_msg[VmbErrorInvalidAccess] = "Invalid access.";
    error_msg[VmbErrorBadParameter] = "Bad parameter.";
    error_msg[VmbErrorStructSize] = "Wrong DLL version.";
    error_msg[VmbErrorWrongType] = "Wrong type.";
    error_msg[VmbErrorInvalidValue] = "Invalid value.";
    error_msg[VmbErrorTimeout] = "Timeout.";
    error_msg[VmbErrorOther] = "TL error.";
    error_msg[VmbErrorInvalidCall] = "Invalid call.";
    error_msg[VmbErrorNoTL] = "TL not loaded.";
    error_msg[VmbErrorNotImplemented] = "Not implemented.";
    error_msg[VmbErrorNotSupported] = "Not supported.";
    error_msg[VmbErrorResources] = "Resource not available.";
    error_msg[VmbErrorInternalFault] = "Unexpected fault in VmbApi or driver.";
    error_msg[VmbErrorMoreData] = "More data returned than memory provided.";

    std::map<VmbErrorType, std::string>::const_iterator iter = error_msg.find(error);
    if (error_msg.end() != iter)
    {
      return iter->second;
    }
    return "Unsupported error code passed.";
  }

  std::string interfaceToString(VmbInterfaceType interfaceType)
  {
    switch (interfaceType)
    {
      case VmbInterfaceFirewire:
        return "FireWire";
        break;
      case VmbInterfaceEthernet:
        return "GigE";
        break;
      case VmbInterfaceUsb:
        return "USB";
        break;
      default:
        return "Unknown";
    }
  }

  std::string accessModeToString(VmbAccessModeType modeType)
  {
    if (modeType & VmbAccessModeFull)
      return "Read and write access";
    else if (modeType & VmbAccessModeRead)
      return "Only read access";
    else if (modeType & VmbAccessModeConfig)
      return "Device configuration access";
    else if (modeType & VmbAccessModeLite)
      return "Device read/write access without feature access (only addresses)";
    else if (modeType & VmbAccessModeNone)
      return "No access";
    else
      return "Undefined access";
  }

#ifdef __GNUC__
  // Customized raw buffer to STL vector converter without memory copy
  class ImageData : public std::vector<uint8_t> {
   public:
    ImageData(uint8_t* begin, uint8_t* end){
      _M_impl._M_start = begin;
      _M_impl._M_finish = end;
      _M_impl._M_end_of_storage = _M_impl._M_finish;
    }

    ~ImageData(){
      _M_impl._M_start = nullptr;
      _M_impl._M_finish = nullptr;
      _M_impl._M_end_of_storage = nullptr;
    }
  };
#else
  // I wonder how Clang handles this, but we will reroute this back to default for now.
  using ImageData = std::vector<uint8_t>;
#endif

  bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::msg::Image& image)
  {
    VmbPixelFormatType pixel_format;
    vimba_frame_ptr->GetPixelFormat(pixel_format);

    // NOTE: YUV formats not tested.
    // YUV444 format not enabled yet for pre-humble compatibility.
    const std::unordered_map<VmbPixelFormatType, std::string> pixel_format_map = {
      {VmbPixelFormatMono8,         sensor_msgs::image_encodings::MONO8},
      {VmbPixelFormatMono10,        sensor_msgs::image_encodings::MONO16},
      {VmbPixelFormatMono12,        sensor_msgs::image_encodings::MONO16},
      {VmbPixelFormatMono12Packed,  sensor_msgs::image_encodings::MONO16},
      {VmbPixelFormatMono14,        sensor_msgs::image_encodings::MONO16},
      {VmbPixelFormatMono16,        sensor_msgs::image_encodings::MONO16},
      {VmbPixelFormatBayerGR8,      sensor_msgs::image_encodings::BAYER_GRBG8},
      {VmbPixelFormatBayerRG8,      sensor_msgs::image_encodings::BAYER_RGGB8},
      {VmbPixelFormatBayerGB8,      sensor_msgs::image_encodings::BAYER_GBRG8},
      {VmbPixelFormatBayerBG8,      sensor_msgs::image_encodings::BAYER_BGGR8},
      {VmbPixelFormatBayerGR10,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerRG10,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerGB10,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerBG10,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerGR12,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerRG12,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerGB12,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerBG12,     sensor_msgs::image_encodings::TYPE_16SC1},
      {VmbPixelFormatBayerGR12Packed, sensor_msgs::image_encodings::TYPE_32SC4},
      {VmbPixelFormatBayerRG12Packed, sensor_msgs::image_encodings::TYPE_32SC4},
      {VmbPixelFormatBayerGB12Packed, sensor_msgs::image_encodings::TYPE_32SC4},
      {VmbPixelFormatBayerBG12Packed, sensor_msgs::image_encodings::TYPE_32SC4},
      {VmbPixelFormatBayerGR16,     sensor_msgs::image_encodings::BAYER_GRBG16},
      {VmbPixelFormatBayerRG16,     sensor_msgs::image_encodings::BAYER_RGGB16},
      {VmbPixelFormatBayerGB16,     sensor_msgs::image_encodings::BAYER_GBRG16},
      {VmbPixelFormatBayerBG16,     sensor_msgs::image_encodings::BAYER_BGGR16},
      {VmbPixelFormatRgb8,          sensor_msgs::image_encodings::RGB8},
      {VmbPixelFormatBgr8,          sensor_msgs::image_encodings::BGR8},
      {VmbPixelFormatRgba8,         sensor_msgs::image_encodings::RGBA8},
      {VmbPixelFormatBgra8,         sensor_msgs::image_encodings::BGRA8},
      {VmbPixelFormatRgb12,         sensor_msgs::image_encodings::TYPE_16UC3},
      {VmbPixelFormatRgb16,         sensor_msgs::image_encodings::TYPE_16UC3},
      {VmbPixelFormatYuv422,        sensor_msgs::image_encodings::YUV422},
      //{VmbPixelFormatYuv444,        sensor_msgs::image_encodings::NV24},
    };
    std::string encoding;
    try {
      encoding = pixel_format_map.at(pixel_format);
    } catch (const std::out_of_range& e) {
      RCLCPP_WARN(logger_, "Received frame with unsupported pixel format %d", pixel_format);
      return false;
    }

    // Acquire the raw image
    VmbUchar_t* buffer_ptr;
    VmbErrorType err = vimba_frame_ptr->GetImage(buffer_ptr);
    if (VmbErrorSuccess != err) {
      RCLCPP_ERROR_STREAM(logger_, "Could not GetImage. "
                                       << "\n Error: " << errorCodeToMessage(err));
      return false;
    }
    VmbUint32_t nSize;
    vimba_frame_ptr->GetImageSize(nSize);
    image.data = ImageData(buffer_ptr, buffer_ptr + nSize);
    // image is already filled. Fill in the metadata
    vimba_frame_ptr->GetHeight(image.height);
    vimba_frame_ptr->GetWidth(image.width);
    image.step = nSize / image.height;
    image.is_bigendian = 0;
    return true;
  }

private:
  VimbaSystem& vs;
  rclcpp::Logger logger_;

  void listAvailableCameras()
  {
    RCLCPP_INFO(logger_, "Searching for cameras ...");
    CameraPtrVector cameras;
    if (VmbErrorSuccess == vs.Startup())
    {
      if (VmbErrorSuccess == vs.GetCameras(cameras))
      {
        for (const auto& camera : cameras)
        {
          std::string strID;
          std::string strName;
          std::string strModelname;
          std::string strSerialNumber;
          std::string strInterfaceID;
          VmbInterfaceType interfaceType;
          VmbAccessModeType accessType;

          VmbErrorType err = camera->GetID(strID);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get camera ID. Error code: " << err << "]");
          }

          err = camera->GetName(strName);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get camera name. Error code: " << err << "]");
          }

          err = camera->GetModel(strModelname);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get camera mode name. Error code: " << err << "]");
          }

          err = camera->GetSerialNumber(strSerialNumber);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get camera serial number. Error code: " << err << "]");
          }

          err = camera->GetInterfaceID(strInterfaceID);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get interface ID. Error code: " << err << "]");
          }

          err = camera->GetInterfaceType(interfaceType);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get interface type. Error code: " << err << "]");
          }

          err = camera->GetPermittedAccess(accessType);
          if (VmbErrorSuccess != err)
          {
            RCLCPP_ERROR_STREAM(logger_, "[Could not get access type. Error code: " << err << "]");
          }

          RCLCPP_INFO_STREAM(logger_, "Found camera named " << strName << ":");
          RCLCPP_INFO_STREAM(logger_, " - Model Name     : " << strModelname);
          RCLCPP_INFO_STREAM(logger_, " - Camera ID      : " << strID);
          RCLCPP_INFO_STREAM(logger_, " - Serial Number  : " << strSerialNumber);
          RCLCPP_INFO_STREAM(logger_, " - Interface ID   : " << strInterfaceID);
          RCLCPP_INFO_STREAM(logger_, " - Interface type : " << interfaceToString(interfaceType));
          RCLCPP_INFO_STREAM(logger_, " - Access type    : " << accessModeToString(accessType));
        }
      }
      else
      {
        RCLCPP_WARN(logger_, "Could not get cameras from Vimba System");
      }
    }
    else
    {
      RCLCPP_WARN(logger_, "Could not start Vimba System");
    }
  }
};
}  // namespace avt_vimba_camera
#endif
