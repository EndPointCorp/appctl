#include "ros/ros.h"
#include <linux/input.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "evdev_teleport/EvdevEvent.h"
#include "evdev_teleport/EvdevEvents.h"

const char* DEVICE_PATH_PARAM = "device_path";
const double SLEEP_DURATION = 0.0001; // seconds

int main(int argc, char** argv) {

  /* initialize ros */

  ros::init(argc, argv, "evdev_teleport_sender");

  ros::NodeHandle n("~");

  /* open the device */

  std::string device_path;
  int device_fd;

  if (n.getParam(DEVICE_PATH_PARAM, device_path)) {
    ROS_INFO("Using device: %s", device_path.c_str());
    n.deleteParam(DEVICE_PATH_PARAM);
  } else {
    ROS_ERROR("Private parameter 'device_path' must be set");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  if ((device_fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK)) < 0) {
    perror("opening the file you specified");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  /* advertise the topic */

  ros::Publisher evdev_pub =
    n.advertise<evdev_teleport::EvdevEvents>("/evdev_teleport/event", 1);

  /* begin relaying from the device to the topic */

  evdev_teleport::EvdevEvents events_msg;

  while(ros::ok()) {
    struct input_event ev;
    struct input_event *event_data = &ev;
    evdev_teleport::EvdevEvent event_msg;

    int num_read = read(device_fd, event_data, sizeof(ev));

    if (sizeof(ev) != num_read) {
      ros::Duration(SLEEP_DURATION).sleep();
      continue;
    }

    if (event_data->type == EV_SYN) {
      if (!events_msg.events.empty()) {
        evdev_pub.publish(events_msg);
        events_msg.events.clear();
      }
      ros::spinOnce();
      continue;
    }

    event_msg.type = event_data->type;
    event_msg.code = event_data->code;
    event_msg.value = event_data->value;
    events_msg.events.push_back(event_msg);

    ROS_DEBUG(
      "got type: %d code: %d value: %d\n",
      event_msg.type, event_msg.code, event_msg.value
    );
  }
}
