#include "ros/ros.h"
#include <string>

#include "uinput_device.h"

const char* DEVICE_NAME_PARAM = "device_name";

int main(int argc, char** argv) {

  /* initialize ros */

  ros::init(argc, argv, "evdev_teleport_receiver");

  ros::NodeHandle n("~");

  /* open the device */

  std::string device_name;
  UinputDevice *uinput_device = new UinputDevice();

  if (n.getParam(DEVICE_NAME_PARAM, device_name)) {
    ROS_DEBUG("Creating device: %s", device_name.c_str());
    n.deleteParam(DEVICE_NAME_PARAM);
  } else {
    ROS_ERROR("Private parameter 'device_name' must be set");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  if (!uinput_device->CreateDevice(device_name)) {
    ROS_ERROR("failed to create uinput device");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  /* begin relaying from the topic to the device */

  ros::Subscriber evdev_sub =
    n.subscribe("/evdev_teleport/event", 1, &UinputDevice::HandleMessage, uinput_device);

  ros::spin();
}
