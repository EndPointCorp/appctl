#include "uinput_device.h"

#include "ros/ros.h"
#include <linux/input.h>
#include <linux/uinput.h>
#include <linux/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "evdev_teleport/EvdevEvent.h"
#include "evdev_teleport/EvdevEvents.h"

UinputDevice::UinputDevice() {}

/*
 * Enables an event type and all codes on a uinput device that is still
 * being initialized.
 */
bool UinputDevice::EnableCodes(int fd, int typeBits, int codeBits, int codeMax) {
  int status;

  status = ioctl(fd, UI_SET_EVBIT, typeBits);
  if (status != 0) {
    ROS_ERROR_STREAM("failed to enable type " << typeBits);
    return false;
  }
  for (int i = 0; i < codeMax; i++) {
    status = ioctl(fd, codeBits, i);
    if (status != 0) {
      ROS_ERROR_STREAM("failed to enable type " << typeBits << " code " << i);
      return false;
    }
  }
}

/*
 * Creates a uinput device.
 * Returns the file descriptor for writing events to the device.
 */
bool UinputDevice::CreateDevice(const std::string dev_name) {
  struct uinput_user_dev uidev;
  int fd;
  int status;

  // poc hardcoded <hax>
  int id_vendor = 0x0001;
  int id_product = 0x0002;
  int version = 1;
  int abs_min[] = { -1024, -1024 };
  int abs_max[] = { 1024, 1024 };
  // </hax>

  // open the special uinput device
  fd = open(UinputDeviceConstants::UINPUT_PATH, O_WRONLY | O_NONBLOCK);
  if (fd < 0) {
    perror("opening the uinput node");
    return false;
  }

  // enable some codes
  EnableCodes(fd, EV_REL, UI_SET_RELBIT, REL_MAX);
  EnableCodes(fd, EV_ABS, UI_SET_ABSBIT, ABS_MAX);
  EnableCodes(fd, EV_KEY, UI_SET_KEYBIT, KEY_MAX);
  EnableCodes(fd, EV_MSC, UI_SET_MSCBIT, MSC_MAX);

  // initialize the user device struct
  memset(&uidev, 0, sizeof(uidev));

  // set the device name
  strncpy(uidev.name, dev_name.c_str(), UINPUT_MAX_NAME_SIZE);

  // set more device attributes
  uidev.id.bustype = BUS_VIRTUAL;
  uidev.id.vendor = (__u16)id_vendor;
  uidev.id.product = (__u16)id_product;
  uidev.id.version = (__u16)version;

  // set ABS min/max values
  /*
  for (int i = 0; i < sizeof(abs_min); i++) {
    uidev.absmin[i] = abs_min[i];
  }
  for (int i = 0; i < sizeof(abs_max); i++) {
    uidev.absmax[i] = abs_max[i];
  }
  */

  // write the device information
  status = write(fd, &uidev, sizeof(uidev));
  if (status != sizeof(uidev)) {
    ROS_ERROR("error writing to uinput device");
    return false;
  }

  // create the device
  status = ioctl(fd, UI_DEV_CREATE);
  if (status != 0) {
    ROS_ERROR("error on ioctl UI_DEV_CREATE");
    return false;
  }

  ROS_DEBUG(
    "created device: %s with vendor: %d product: %d version: %d",
    uidev.name, uidev.id.vendor, uidev.id.product, uidev.id.version
  );

  // fd is now a handle for the user device
  fd_ = fd;
  return true;
}

/*
 * Handles a ROS message containing a vector of events.
 */
void UinputDevice::HandleMessage(const evdev_teleport::EvdevEvents::Ptr& msg) {
  for (int i = 0; i < msg->events.size(); i++) {

    evdev_teleport::EvdevEvent ev = msg->events[i];

    if (!WriteEvent(ev.type, ev.code, ev.value)) {
      ROS_ERROR("Error writing an event to the device");
      ros::shutdown();
    }
  }

  // force an EV_SYN after each message
  if (!WriteEvent(EV_SYN, SYN_REPORT, 0)) {
    ROS_ERROR("Error writing a SYN event");
    ros::shutdown();
  }
}

/*
 * Writes an event to the virtual device.
 * Returns success.
 */
bool UinputDevice::WriteEvent(__u16 type, __u16 code, __s32 value) {
  struct input_event ev;

  memset(&ev, 0, sizeof(ev));

  ev.type = type;
  ev.code = code;
  ev.value = value;

  int num_wrote = write(fd_, &ev, sizeof(ev));

  if (num_wrote != sizeof(ev)) {
    return false;
  }

  ROS_DEBUG(
    "wrote type: %d code: %d value: %d",
    ev.type, ev.code, ev.value
  );

  return true;
}
