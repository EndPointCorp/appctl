#!/usr/bin/env python

import rospy

import appctl_support
from appctl.msg import Mode
from appctl.srv import Query

GSTREAMER_LAUNCHER = 'gst-launch'

PLAYBACK_ARGS = [
  'tcpclientsrc', 'host={playback_host}', 'port={playback_port}',
  '!',
  'capsfilter', 'caps=video/x-h264,width=1920, height=1080,framerate=60/1, pixel-aspect-ratio=1/1, stream-format=byte-stream, alignment=au, level=4, profile=main',
  '!',
  'ffdec_h264',
  '!',
  'xvimagesink', 'display={display}'
]

def main():
  rospy.init_node('mirror_playback', anonymous=True)

  modes = rospy.get_param('~modes').split(',')
  server_host = rospy.get_param('~host')
  server_port = str(rospy.get_param('~port', 4953))
  display = rospy.get_param('~display', ':0')

  rospy.loginfo('running on modes {}'.format(modes))

  cmd = [GSTREAMER_LAUNCHER]
  args = map(lambda arg: arg.format(
    playback_host = server_host,
    playback_port = server_port,
    display = display
  ), PLAYBACK_ARGS)

  cmd.extend(args)

  proc_controller = appctl_support.ProcController(cmd)
  mode_handler = appctl_support.ModeHandler(modes, proc_controller)

  rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_mode_msg)

  # TODO(mv): check /appctl/query

  rospy.spin()

  mode_handler.shutdown()

if __name__=='__main__':
  main()
