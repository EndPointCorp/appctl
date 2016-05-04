import rospy
import threading
from controller import BaseController
from proc_runner import ProcRunner
from std_msgs.msg import String
from appctl.srv import NodeQuery


class ProcController(BaseController):
    """
    Controls startup and shutdown of a ProcRunner.
    """
    def __init__(self, cmd, shell=False, spawn_hooks=None):
        if spawn_hooks is None:
            spawn_hooks = []
        self.cmd = cmd
        self.shell = shell
        self.started = False
        self.watcher = None
        self.spawn_hooks = spawn_hooks
        self.status_lock = threading.Lock()
        self.request_lock = threading.Lock()
        root = '/appctl' + rospy.get_name()
        self.ros_control = rospy.Subscriber(
            '%s/control' % root,
            String,
            self.handle_request
        )
        self.node_query = rospy.Service('%s/query' % root, NodeQuery,
                                        self.handle_query)

        # Always stop on rospy shutdown.
        rospy.on_shutdown(self.stop)

    def start(self, *args, **kwargs):
        with self.status_lock:
            if self.started:
                return
            self.started = True
            self.watcher = ProcRunner(self.cmd, shell=self.shell,
                                      spawn_hooks=self.spawn_hooks)
            self.watcher.daemon = False
            self.watcher.start()

    def stop(self, *args, **kwargs):
        with self.status_lock:
            if not self.started:
                return
            self.started = False
            self.watcher.shutdown()
            self.watcher = None

    def add_spawn_hook(self, spawn_hook):
        """
        Adds a spawn hook to the current list of spawn hooks

        If there is already a watcher, also calls down to that object
        to add the new spawn hook as well
        """
        self.spawn_hooks.append(spawn_hook)
        if self.watcher:
            self.watcher.add_spawn_hook(spawn_hook)

    def get_pid(self):
        """
        Return process id, or 0 to signify error
        """
        with self.status_lock:
            if self.watcher:
                return self.watcher.get_pid()
            return None

    def handle_request(self, msg):
        # lock so we don't run into race conditions by sending start
        # and stop messages around the same time...
        with self.request_lock:
            request = msg.data
            if request == 'start':
                self.start()
            elif request == 'stop':
                self.stop()
            elif request =='restart':
                self.stop()
                self.start()

    def handle_query(self, req):
        request = req.req
        if request == 'get_pid':
            return str(self.get_pid() or 0)
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
