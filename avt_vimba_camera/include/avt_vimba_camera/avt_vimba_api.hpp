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

#include <VmbCPP/VmbCPP.h>
#include <VmbImageTransform/VmbTransform.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>

#include <string>
#include <map>

using VmbCPP::CameraPtr;
using VmbCPP::FramePtr;
using VimbaSystem = VmbCPP::VmbSystem;
using VmbInterfaceType = VmbTransportLayerType;

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
      // case VmbInterfaceFirewire:
      case VmbTransportLayerTypeIIDC:
        return "FireWire";
        break;
      // case VmbInterfaceEthernet:
      case VmbTransportLayerTypeGEV:
        return "GigE";
        break;
      // case VmbInterfaceUsb:
      case VmbTransportLayerTypeUVC:
        return "USB";
        break;
      case VmbTransportLayerTypeU3V:
        return "USB3";
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
    else if (modeType & VmbAccessModeUnknown)
      return "Device configuration access";
    else if (modeType & VmbAccessModeExclusive)
      return "Device read/write access without feature access (only addresses)";
    else if (modeType & VmbAccessModeNone)
      return "No access";
    else
      return "Undefined access";
  }

  bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::msg::Image& image)
  {
    VmbPixelFormatType pixel_format;
    vimba_frame_ptr->GetPixelFormat(pixel_format);
    VmbUint32_t width, height;
    vimba_frame_ptr->GetWidth(width);
    vimba_frame_ptr->GetHeight(height);

    // Acquire the raw image
    VmbUint8_t* buffer_ptr = image.data.data();
    VmbErrorType err = vimba_frame_ptr->GetImage(buffer_ptr);
    if (VmbErrorSuccess != err) {
      RCLCPP_ERROR_STREAM(logger_, "Could not GetImage. "
                                       << "\n Error: " << errorCodeToMessage(err));
      return false;
    }

    VmbImage vmb_img;
    vmb_img.Size = sizeof(vmb_img);
    vmb_img.Data = buffer_ptr;
    // determine pixel format
    VmbError_t err2 = VmbSetImageInfoFromPixelFormat(pixel_format, width, height, &vmb_img);
    if (err2 != VmbErrorSuccess) {
      RCLCPP_ERROR_STREAM(logger_, "VmbSetImageInfoFromPixelFormat: error "
                                      << errorCodeToMessage(static_cast<VmbErrorType>(err2)));
      return false;
    }

    // perform debayering with Vimba's own API
    if (isBayeredImage(pixel_format)){
      // first debayer, then fill the image
      VmbTransformInfo vmb_transform_info;
      vmb_transform_info.TransformType = VmbTransformTypeDebayerMode;
      if (VmbErrorSuccess != VmbSetDebayerMode(VmbDebayerMode2x2, &vmb_transform_info)) {
        RCLCPP_ERROR(logger_, "VmbSetDebayerMode failed");
        return false;
      }
      VmbImage vmb_img_debayered;
      vmb_img_debayered.Size = sizeof(vmb_img_debayered);
      image.encoding = sensor_msgs::image_encodings::RGB8;
      // Need to swap out the bayered data in the output image
      std::vector<uint8_t> old_data{};
      image.data.swap(old_data);
      vmb_img.Data = old_data.data();
      // prepare to fill the new debayered data
      vmb_img_debayered.Data = image.data.data();
      if (VmbErrorSuccess != VmbSetImageInfoFromPixelFormat(VmbPixelFormatRgb8, width, height, &vmb_img_debayered)){
        RCLCPP_ERROR(logger_, "VmbSetImageInfoFromPixelFormat failed");
        return false;
      }
      image.height = height;
      image.width = width;
      VmbUint32_t step = vmb_img_debayered.ImageInfo.Stride * (vmb_img_debayered.ImageInfo.PixelInfo.BitsPerPixel / 8);
      image.step = step;
      image.is_bigendian = 0;
      if (VmbErrorSuccess != VmbImageTransform(&vmb_img, &vmb_img_debayered, &vmb_transform_info, 1)){
        RCLCPP_ERROR(logger_, "VmbImageTransform failed");
        return false;
      }
      return true;
    } else {
      // image is already filled. Fill in the metadata
      VmbUint32_t step = vmb_img.ImageInfo.Stride * (vmb_img.ImageInfo.PixelInfo.BitsPerPixel / 8);
      image.height = height;
      image.width = width;
      image.step = step;
      image.is_bigendian = 0;
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
      image.encoding = encoding;
      return true;
    }
  }

private:
  VimbaSystem& vs;
  rclcpp::Logger logger_;

  static bool isBayeredImage(const VmbPixelFormatType& pixfmt){
    if (!(pixfmt & VmbPixelMono)) return false;
    auto lsb16 = pixfmt & 0xFFFF;
    if (lsb16 >= 0x0008 && lsb16 <= 0x0013) return true;
    if (lsb16 >= 0x002A && lsb16 <= 0x0031) return true;
    if (lsb16 >= 0x0052 && lsb16 <= 0x0059) return true;
    return false;
  }

  void listAvailableCameras()
  {
    RCLCPP_INFO(logger_, "Searching for cameras ...");
    CameraPtrVector cameras;
    //if (VmbErrorSuccess == vs.Startup())
    //{
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
    /*
    } else {
      RCLCPP_WARN(logger_, "Could not start Vimba System");
    }
    */
  }
};
}  // namespace avt_vimba_camera
#endif
