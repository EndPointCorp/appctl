from pynag import Model
import rospy

class PortalStatus():
    def __init__(self):
        self.all_hosts = Model.Host.objects.all

    def get_status(self):
        if self._all_hosts_up() and self._all_services_ok():
            return 'on'
        if self._all_hosts_up() and (not self._all_services_ok()):
            return 'malfunction'
        if not self._all_hosts_up():
            return 'off'

    def _all_hosts_up(self):
        """ Check if SSH service state for all hosts is OK"""
        for hostname in self.all_hosts:
            try:
                status = s.get_servicestatus(host_name=hostname,
                                             service_description='SSH')
                if status['current_state'] == '0':
                    return False
            except Exception, e:
                # we probably hit 'generic-host' which has no services
                pass
        return True

    def _all_services_ok(self):
        """ check if Portal services are OK"""
        services = Model.Service.objects.filter(service_description__startswith='PORTAL', register='1')
        for service in services:
            try:
                if service.get_current_status()['current_state'] != '0':
                    return False
            except Exception, e:
                # we've probably hit unconfigured service
                pass

        return True

if __name__ == '__main__':
    ps = PortalStatus()
    print ps.get_status()
