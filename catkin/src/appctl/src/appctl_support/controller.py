# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

from abc import ABCMeta, abstractmethod

class BaseController():
    __metaclass__ = ABCMeta

    @abstractmethod
    def start(self, *args, **kwargs):
        pass

    @abstractmethod
    def stop(self, *args, **kwargs):
        pass
