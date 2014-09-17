#ifndef _UINPUT_DEVICE_H_
#define _UINPUT_DEVICE_H_

#include <linux/types.h>
#include <string>

#include "evdev_teleport/EvdevEvents.h"

namespace UinputDeviceConstants {
  const char* UINPUT_PATH = "/dev/uinput";
}

class UinputDevice {
  public:
    UinputDevice();

    bool CreateDevice(const std::string dev_name);

    void HandleMessage(const evdev_teleport::EvdevEvents::Ptr& msg);
    bool WriteEvent(__u16 type, __u16 code, __s32 value);

  private:
    static bool EnableCode(int fd, int codeBits, int code);

    int fd_;
};

#endif // _UINPUT_DEVICE_H_
