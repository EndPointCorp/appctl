appctl
------

A package of support software for controlling which processes are running on an interactive kiosk.  The core concept of appctl is that each application configuration can run in any number of "Modes," with only one "Mode" active at any given moment.

### The appctl\_support Library

A set of Python support modules for implementing basic application control nodes.  `ModeHandler` is a high level abstraction of process control, while `ProcController` and `ProcRunner` are available for advanced use.

##### appctl\_support.ModeHandler

Wraps a `BaseController` subclass and provides a mode message handler.  This is a convenience class for writing nodes that start something when certain modes are activated.

Initialize with a list of mode strings and a controller instance.  Use `begin_handling_modes()` after initializing your node to start tracking mode changes.

##### appctl\_support.ProcController

Controls startup and shutdown of a `ProcRunner`.  This class spawns `ProcRunner` instances as needed, ensuring that only one is active at a time.

Initialize with a list of command+args and use the `start()` and `stop()` methods to put the `ProcController` in the desired state.

##### appctl\_support.ProcRunner

A `Thread` that launches and manages an external subprocess.  `ProcRunner` will respawn its subprocess if it dies and can be `shutdown()` at any time, instantly terminating its subprocess.

Initialize with a list of command+args, start with `start()`.  Inherits from `threading.Thread`.  Automatically makes itself a `daemon` `Thread`.

### Writing Application Control Nodes

The philosophy behind appctl is that the units of application control should be flat and configuration-driven.  A succinct control nodes will do two things:

1. Build up the command and arguments for running the application from its private ROS parameters.

2. Run the application while a recognized mode is set.

Basically, the only work to be done by each control node is initializing the ROS node and turning its parameters into a command which is then fed into a `ModeHandler` class.  That `ModeHandler` will request the current mode and subscribe to future mode changes for business times.

Consider this example netcat control node which runs a netcat listener on a configurable port while in a recognized mode.

    #!/usr/bin/env python
    """A simple appctl node for /bin/nc"""
    
    import rospy
    from appctl_support import ProcController
    from appctl_support import ModeHandler
    
    if __name__=='__main__':
        rospy.init_node('netcat_listener', anonymous=True)
    
        # Modes should be a comma-separated list.
        modes = rospy.get_param('~modes').split(',')
    
        # A required port parameter.
        port = rospy.get_param('~port')
    
        # Build the command+args list.
        cmd = ['/bin/nc', '-l', str(port)]
    
        # Instantiate the ProcController.
        proc_controller = ProcController(cmd)
    
        # Create and subscribe the ModeHandler.
        mode_handler = ModeHandler(modes, proc_controller)
    
        # Begin handling mode changes, starting with a query.
        mode_handler.begin_handling_modes()
    
        rospy.spin()
        mode_handler.shutdown()

You might configure this node for roslaunch like so, listening on port 33333 while in the "attentive" or "intrigued" modes.

    <launch>
        <node name="netcat_listener_33333" pkg="netcat" type="listener.py">
            <param name="modes" value="attentive,intrigued"/>
            <param name="port" value="33333"/>
        </node>
    </launch>

You could test all of this by publishing the "attentive" mode:

    $ rostopic pub /appctl/mode appctl/Mode attentive

Observe that the `nc` process is running, then go into an unrecognized mode:

    $ rostopic pub /appctl/mode appctl/Mode asleep

Now the process should be gone.

### The controller.py Node

This node is intended as a state manager for appctl mode selection.  Since Mode changes can come from any publisher, ROS latching is impossible for state-keeping.  The AppController will (optionally) set a default initial state and provide a service for other nodes to query as part of their startup.

##### Parameters

* `~initial_mode` : string : Initial Mode.

##### Routes

* `/appctl/mode` : `appctl.msg.Mode` : The route on which Mode changes are published by any source.

##### Services

* `/appctl/query` : `appctl.srv.Query` : A service for querying the currently selected Mode.

You can query the current mode for troubleshooting:

    $ rosservice call /appctl/query

### Tests

Run the tests with `catkin_make`:

    $ catkin_make test

Or with `rostest`:

    $ rostest appctl test_all.test

