from abc import ABCMeta, abstractmethod


class BaseController():
    """Abstract base class for appctl controllers."""
    __metaclass__ = ABCMeta

    @abstractmethod
    def start(self, *args, **kwargs):
        pass

    @abstractmethod
    def stop(self, *args, **kwargs):
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
