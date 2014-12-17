"""
Browser platform tests (webgl, drivers etc).

"""

import helpers
from base import TestBase
from exception import ConfigException


class TestPlatform(TestBase):
    """
    Following tests will check whether:
        * version of the browser matches the version in config.json

    Check:
        * gpu_data.clientInfo.version => Chrome/37.0.2062.120
        * gpu_data.gpuInfo.basic_info[n] => description : 'Direct Rendering'

    Log:
        * gpu_data.gpuInfo.basic_info[n] => description : GL_VERSION

    """

    def setup_method(self, method):
        super(TestPlatform, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["generic"])

    def test_get_chrome_gpu_data(self):
        """
        Check if the chrome supports GPU.

        """
        self.browser.get(self.config["chrome_gpu_url"])
        gpu_js = 'window.setTimeout("browserBridge = new gpu.BrowserBridge();\
                 ",1000); return browserBridge'
        self.chrome_gpu_data = self.browser.execute_script(gpu_js)

        chrome_version = None
        try:
            chrome_version = self.config["chromes"]["generic"]["version"]
        except ConfigException, e:
            print "%s => you must provide chrome version in config.json" % e
            print "e.g. config['chrome']['version'] == 'Chrome/30'"

        config_chrome_version = str(self.chrome_gpu_data['clientInfo']['version'].split('.')[0])
        direct_rendering_enabled = \
            helpers.filter_list_of_dicts(self.chrome_gpu_data['gpuInfo']['basic_info'],
                                         'Direct rendering',
                                         'Yes')
        assert direct_rendering_enabled is True
        assert chrome_version == config_chrome_version