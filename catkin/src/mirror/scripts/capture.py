#!/usr/bin/env python

import rospy

import appctl_support

GSTREAMER_LAUNCHER = 'gst-launch'

CAPTURE_ARGS = [
  'ximagesrc', 'display-name={display}', 'show-pointer=false', 'use-damage=false',
  '!',
  'videoscale',
  '!',
  'video/x-raw-rgb,width=1920,height=1080,framerate=60/1',
  '!',
  'ffmpegcolorspace',
  '!',
  'x264enc', 'tune=zerolatency', 'speed-preset=medium', 'threads=4',
    'sliced-threads=true', 'bitrate=6000', 'byte-stream=true',
  '!',
  'tcpserversink', 'port={server_port}'
]

def main():
  rospy.init_node('mirror_capture', anonymous=True)

  modes = rospy.get_param('~modes').split(',')
  server_port = str(rospy.get_param('~port', 4953))
  display = rospy.get_param('~display', ':0')

  rospy.loginfo('running on modes {}'.format(modes))

  cmd = [GSTREAMER_LAUNCHER]

  args = map(lambda arg: arg.format(
    server_port = server_port,
    display = display
  ), CAPTURE_ARGS)

  cmd.extend(args)

  proc_controller = appctl_support.ProcController(cmd)
  mode_handler = appctl_support.ModeHandler(modes, proc_controller)

  mode_handler.begin_handling_modes()

  rospy.spin()

  mode_handler.shutdown()

if __name__=='__main__':
  main()
