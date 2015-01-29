#!/usr/bin/env python

"""
CI Jenkins interaction helper script.
    -first prints stats on specified jenkins projects/jobs
    -runs projects
    -avoids manual interactions with several Jenkins projects

https://pypi.python.org/pypi/jenkinsapi
    API could get/set jobs configuration (will be useful on a mass changes)

    other interesting API:
        self.server.build_job
        job = self.server.get_job(job_name)
        dir(job)
        job.get_build_ids() # returns iterator
        job.get_build_triggerurl()
        job.get_last_build()
        job.get_last_buildnumber()
        job.is_running()
        job.is_queued_or_running()
        job.poll() # return None if not running
        job.invoke(securitytoken=None, block=False, skip_if_running=False)
        job.get_build(275)
        job.get_last_buildnumber()
        job.get_last_completed_buildnumber()
        job.get_last_failed_buildnumber()
        job.get_last_good_buildnumber()
        job.get_last_stable_buildnumber()

"""


import os
import sys
import time
import datetime
from optparse import OptionParser, TitledHelpFormatter

from jenkinsapi.jenkins import Jenkins
from jenkinsapi.custom_exceptions import NoBuildData


JENKINS_URL="http://lg-ci-01.endpoint.com:8080"


# jenkins job/project names
JOBS = [
    "Selenium Portal development",
    "Selenium Portal development public_beta",
    "Selenium Portal development public_stable",
    "Selenium Portal MTV development public_beta",
    "Selenium Portal MTV development public_stable",
    "Selenium Portal MTV development sandbox_beta",
    "Selenium Portal MTV development sandbox_stable",
    "Selenium Portal MTV topic_selenium public_beta",
    "Selenium Portal MTV topic_selenium public_stable",
    "Selenium Portal MTV topic_selenium sandbox_beta",
    "Selenium Portal MTV topic_selenium sandbox_stable",
    "Selenium Portal topic_selenium"
]


class JenkinsAccess(object):
    def __init__(self, user_name, token):
        self.server = Jenkins(JENKINS_URL, username=user_name, password=token)

    def access_test(self):
        print "Jenkins server '%s' version access test ..." % JENKINS_URL
        print "Jenkins server version: '%s'\n" % self.server.version

    def print_jobs_stats(self):
        """
        Print Jenkins job statistics:
            job name, last job status, last job id, last failed job id
        
        """
        fail, success = 0, 0
        format = "%-50s %-15s %s" 
        print format % ("JOB NAME", "LAST", "LAST FAILED")
        print 80 * '-'
        for job_name in JOBS:
            job = self.server.get_job(job_name)
            last_id = job.get_last_completed_buildnumber()
            try:
                last_success_id = job.get_last_good_buildnumber()
            except NoBuildData as ex:
                last_success_id = -1
            try:
                last_fail_id = job.get_last_failed_buildnumber()
            except NoBuildData as ex:
                last_fail_id = "N/A"
            
            if last_id == last_success_id:
                last_status = "SUCCESS"
                success += 1
            else:
                last_status = "FAIL"
                fail += 1
            print format % (job_name,
                            "%-8s #%s" % (last_status, last_id),
                            "#%s" % last_fail_id)
        print "SUCCESS: %s" % success
        print "FAIL:    %s" % fail

    def run_jobs(self):
        """
        Invoke specified Jenkins project, wait until all are completed.

        """
        print "\n\nRunning jobs ...\n"
        running = []
        for job_name in JOBS:
            job = self.server.get_job(job_name)
            if job.is_queued_or_running():
                print "%s: queued or running, skipping ..." % job_name
                continue
            print "%s: invoking job ..." % job_name
            job.invoke(block=False, skip_if_running=True)
            time.sleep(2)
            print "running: %s" % job.is_queued_or_running()
            running.append(job_name)

        while len(running) > 0:
            print "\nWaiting for %s jobs to complete ..." % len(running)
            for job_name in running[:]:
                job = self.server.get_job(job_name)
                if job.is_queued_or_running():
                    if job.is_running():
                        print "%s: running." % job_name
                    else:
                        print "%s: queued." % job_name
                else:
                    print "%s: finished." % job_name
                    running.remove(job_name)
            if len(running) > 0:
                t = 300
                print "Waiting for %s seconds ..." % t
                time.sleep(t)
        print "\nall jobs finished."


def process_cli_arguments(cli_args):
    """
    Process values entered on the command line.
    Return user name, jenkins access token, run flag.

    """
    form = TitledHelpFormatter(width=78)
    usage = "%prog --user_name <user name> --token <token>"
    parser = OptionParser(usage=usage, formatter=form)
    # option, its value
    parser.add_option("-u",
                      "--user_name",
                      dest="user_name",
                      default='',
                      help="Jenkins user name.")
    parser.add_option("-t",
                      "--token",
                      dest="token",
                      default='',
                      help="Jenkins user's access token.")
    parser.add_option("-r",
                      "--run",
                      dest="run",
                      action="store_true",
                      help=("Run specified Jenkins projects / jobs, poll "
                            "the state until completed and print latest "
                            "results."))
    opts, args = parser.parse_args(cli_args)
    if (opts.user_name == '' or opts.token == ''):
        print "Missing mandatory argument(s), try --help"
        sys.exit(1)
    return opts.user_name, opts.token, opts.run


def main():
    user_name, token, run_flag = process_cli_arguments(sys.argv)
    jenkins = JenkinsAccess(user_name, token)
    start = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    jenkins.access_test()
    jenkins.print_jobs_stats()
    if run_flag:
        jenkins.run_jobs()
        print "\n\nStatus after last run:\n"
        jenkins.print_jobs_stats()
    print "START: %s" % start
    print "FINISH: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


if __name__ == "__main__":
    main()
