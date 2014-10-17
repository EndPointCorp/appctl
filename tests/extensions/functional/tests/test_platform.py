"""
Browser platform tests (webgl, drivers etc)
"""

from base import TestBaseGeneric
from base import screenshot_on_error
from base import CHROME_GPU_URL
from base import CONFIG
from helpers import filter_list_of_dicts
from exceptions import ConfigException


class TestPlatform(TestBaseGeneric):
    """
    Following tests will check whether
    - version of the browser
    - matches the version in config.json

    Check:
    gpu_data.clientInfo.version => Chrome/37.0.2062.120
    gpu_data.gpuInfo.basic_info[n] => description : 'Direct Rendering'

    Get:
    gpu_data.gpuInfo.basic_info[n] => description : GL_VERSION
    """

    def __init__(self):
        self.chrome_gpu_data = self.get_chrome_gpu_data()
        print "CHROME GPU DATA {}".format(self.chrome_gpu_data) #removeme
        try:
            self.chrome_version = CONFIG['chrome']['version']
        except ConfigException, e:
            print "%s => you must provide chrome version in config.json" % e
            print "e.g. config['chrome']['version'] == 'Chrome/38.0'"

        if "ignore-gpu-blacklist" in CONFIG["chrome"]["arguments"]:
            self.gpu_present = False
        else:
            self.gpu_present = True

    def get_chrome_gpu_data(self):
        self.browser.get(CHROME_GPU_URL)
        return self.browser.execute_script('gpu_data = new gpu.BrowserBridge();\
                                                            return gpu_data;')

    @screenshot_on_error
    def direct_rendering_enabled(self):
        pass

    @screenshot_on_error
    def chrome_version_is_correct(self):
        assert self.chrome_gpu_data['clientInfo']['version'].split('.')[0] is self.chrome_version
