from abc import ABCMeta, abstractmethod


class BaseController(metaclass=ABCMeta):
    """Abstract base class for appctl controllers."""

    @abstractmethod
    def start(self, *args, **kwargs):
        pass

    @abstractmethod
    def stop(self, *args, **kwargs):
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
