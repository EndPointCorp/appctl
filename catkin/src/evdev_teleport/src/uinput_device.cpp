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
#include "std_msgs/Bool.h"

UinputDevice::UinputDevice() {}

/*
 * Shortcut for enabling an event type or code during virtual device setup.
 * Returns true on success.
 */
bool UinputDevice::EnableCode(int fd, int codeBits, int code) {
  if (ioctl(fd, codeBits, code) != 0) {
    ROS_ERROR_STREAM("failed to enable code " << codeBits << ":" << code);
    return false;
  }

  return true;
}

/*
 * Creates a uinput device.
 * Returns the file descriptor for writing events to the device.
 */
bool UinputDevice::CreateDevice(const std::string dev_name) {
  struct uinput_user_dev uidev;
  int fd;
  int status;

  // poc hardcoded <hax> does this stuff matter?
  // doesn't show up in udev, but does in xinput
  int id_vendor = 0x0001;
  int id_product = 0x0002;
  int version = 1;
  // </hax>

  // open the special uinput device
  fd = open(UinputDeviceConstants::UINPUT_PATH, O_WRONLY | O_NONBLOCK);
  if (fd < 0) {
    perror("opening the uinput node");
    return false;
  }

  // enable some codes for the ELO touchscreen <hax>
  // some of these are reported from sniffed touchscreen events
  // others i found on stack
  // http://stackoverflow.com/questions/18544734
  EnableCode(fd, UI_SET_EVBIT, EV_SYN);
  EnableCode(fd, UI_SET_EVBIT, EV_KEY);
  EnableCode(fd, UI_SET_KEYBIT, BTN_TOUCH);
  EnableCode(fd, UI_SET_EVBIT, EV_ABS);
  EnableCode(fd, UI_SET_ABSBIT, ABS_X);
  EnableCode(fd, UI_SET_ABSBIT, ABS_Y);
  EnableCode(fd, UI_SET_ABSBIT, ABS_MT_POSITION_X);
  EnableCode(fd, UI_SET_ABSBIT, ABS_MT_POSITION_Y);
  EnableCode(fd, UI_SET_ABSBIT, ABS_MT_TRACKING_ID);
  EnableCode(fd, UI_SET_ABSBIT, ABS_MT_SLOT);
  // </hax>

  // initialize the user device struct
  memset(&uidev, 0, sizeof(uidev));

  // set the device name
  strncpy(uidev.name, dev_name.c_str(), UINPUT_MAX_NAME_SIZE);

  // set more device attributes
  uidev.id.bustype = BUS_VIRTUAL; // good as any? not visible in udev props
  uidev.id.vendor = (__u16)id_vendor;
  uidev.id.product = (__u16)id_product;
  uidev.id.version = (__u16)version;

  // set ABS min/max values for the ELO touchscreen <hax>
  // x/y axes, find these for your touchscreen with xinput list #ID#
  uidev.absmax[ABS_X] = 32767;
  uidev.absmax[ABS_Y] = 32767;
  uidev.absmax[ABS_MT_POSITION_X] = 32767;
  uidev.absmax[ABS_MT_POSITION_Y] = 32767;
  // ID for tracking unique touches, constant for any touchscreen?
  uidev.absmax[ABS_MT_TRACKING_ID] = 65535;
  // number of touch slots (start at 0)
  // this is also in the output of xinput list #ID#
  uidev.absmax[ABS_MT_SLOT] = 9;
  // </hax>

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
  // by default, start active
  active_ = true;
  return true;
}

/*
 * Getter, returns true if the device is active.
 */
bool UinputDevice::IsActive() {
  return active_;
}

/*
 * Setter, tells the receiver to either handle messages or ignore them.
 */
void UinputDevice::SetActive(bool active) {
  active_ = active;
}

/*
 * Handles a ROS message activating or deactivating the receiver.
 */
void UinputDevice::HandleActivationMessage(const std_msgs::Bool::Ptr& msg) {
  SetActive(msg->data);
  ROS_DEBUG("activation: %s", msg->data ? "true" : "false");
}

/*
 * Handles a ROS message containing a vector of events.
 */
void UinputDevice::HandleEventMessage(
  const evdev_teleport::EvdevEvents::Ptr& msg) {

  // Ignore messages if the receiver is not activated.
  if (!IsActive()) {
    return;
  }

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
