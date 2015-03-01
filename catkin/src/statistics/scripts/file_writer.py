#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import json
import time
import copy
from statistics.srv import SessionQuery
from statistics.msg import Session
from portal_statistics.portal_status import PortalStatus


class GeolocationDataException(Exception):
    pass


class FileWriter:
    """
    - periodically collects session events from /statistics/session service
    - wraps session events with metadata and renders json files to filesystem
    """

    def __init__(self):
        self.node = self._init_node()
        self.glink_output_path = rospy.get_param('~json_output_path', '/tmp')
        self.ep_output_path = rospy.get_param('~json_output_path_ep', '/tmp')
        self.interval = rospy.get_param('~aggregation_interval', 300)
        self.country = rospy.get_param('~country', None)
        self.store_id = rospy.get_param('~store_id', None)
        self.retailer = rospy.get_param('~retailer', None)
        self.glink_session_keys = ["end_ts", "start_ts"]
        self.ep_session_keys = ["end_ts", "start_ts", "application", "mode", "occupancy_triggered"]

        if self.country is None or\
           self.store_id is None or\
           self.retailer is None:
                raise GeolocationDataException("You need to specify proper store metadata")
        self._wait_for_service()
        pass

    def _wait_for_service(self):
        rospy.logdebug("Waiting for the /statistics/session service to become available")
        rospy.wait_for_service('statistics/session')
        rospy.logdebug("Statistics aggretator service has become available")
        pass

    def _call_service(self):
        service_call = rospy.ServiceProxy('statistics/session', SessionQuery)
        response = service_call(erase=True)
        rospy.logdebug("Received response from service: %s" % response)
        return response

    def _sleep_between_requests(self):
        rospy.logdebug("Sleeping for %s seconds before writing stats" % self.interval)
        rospy.sleep(self.interval)
        pass

    def _init_node(self):
        rospy.init_node('statistics')
        pass

    def _finalize_report_data(self, report_contents):
        """
        - fill report_time and end_ts with current time
        - fill start_ts with current_time minus self.interval
        """
        unix_now = int(time.time())
        report_contents['report_time'] = unix_now
        report_contents['start_ts'] = unix_now - self.interval
        report_contents['end_ts'] = unix_now
        report_contents['status'] = PortalStatus().get_status()
        return report_contents

    def _get_status(self):
        return PortalStatus().get_status()

    def _render_glink_stats(self, report_template, sessions):
        for session in sessions:
            session = self._get_session_attributes(session, self.glink_session_keys)
            report_template['sessions'].append(session)

        report_contents = self._finalize_report_data(report_template)
        if report_contents['sessions']:
            self._write_glink_file(report_contents)
        pass

    def _render_endpoint_stats(self, report_template, sessions):
        for session in sessions:
            session = self._get_session_attributes(session, self.ep_session_keys)
            report_template['sessions'].append(session)

        report_contents = self._finalize_report_data(report_template)
        if report_contents['sessions']:
            self._write_endpoint_file(report_contents)
        pass

    def _compose_and_write_json(self, sessions):
        """
        - convert sessions list to json
        - finalize report and write the file
        """
        report_template = {
                           "report_time": 0,
                           "start_ts": 0,
                           "status": self._get_status(),
                           "experience_type": "portal",
                           "end_ts": 0,
                           "metadata": {"country": self.country,
                                        "store_id": self.store_id,
                                        "retailer": self.retailer
                                        },
                           "sessions": []
                           }

        self._render_endpoint_stats(copy.deepcopy(report_template), sessions)
        self._render_glink_stats(copy.deepcopy(report_template), sessions)
        pass

    def _write_glink_file(self, report_contents):
        with open(self.glink_output_path + "/" +  str(int(time.time())) + ".json", 'w') as json_file:
            rospy.loginfo("Writing file %s" % json_file.name)
            json_file.write(json.dumps(report_contents, sort_keys=True, indent=4, separators=(',', ': ')))
            del report_contents
        pass

    def _write_endpoint_file(self, report_contents):
        with open(self.ep_output_path + "/" +  str(int(time.time())) + "_ep.json", 'w') as json_file:
            rospy.loginfo("Writing file %s" % json_file.name)
            json_file.write(json.dumps(report_contents, sort_keys=True, indent=4, separators=(',', ': ')))
            del report_contents
        pass

    def _get_session_attributes(self, session, attributes_list):
        single_session = {}
        for field in attributes_list:
            single_session[field] = session.__getattribute__(field)
            rospy.logdebug("Examining field %s" % field)
        rospy.logdebug("Returning single_session %s" % single_session)
        return single_session

    def _spin(self):
        try:
            response = self._call_service()
        except rospy.service.ServiceException, e:
            rospy.logfatal("Failure while sending requests to /statistics/session service because: %s\
 - this message shouldnt appear anywhere but on ROS shutdown" % e)
            raise
        self._compose_and_write_json(response.sessions)
        pass

    def run(self):
        rospy.logdebug("Starting session statistics service")
        while not rospy.is_shutdown():
            self._sleep_between_requests()
            self._spin()
            pass
        pass

if __name__ == '__main__':
    try:
        FileWriter().run()
    except rospy.ROSInterruptException:
        pass
    except rospy.service.ServiceException:
        pass
