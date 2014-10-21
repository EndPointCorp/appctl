"""
Browser platform tests (webgl, drivers etc)
"""

from base import TestBaseGeneric
from base import CHROME_GPU_URL
from helpers import filter_list_of_dicts
from exception import ConfigException
from base import load_configuration


class TestPlatform(TestBaseGeneric):
    """
    Following tests will check whether
    - version of the browser
    - matches the version in config.json

    Check:
    gpu_data.clientInfo.version => Chrome/37.0.2062.120
    gpu_data.gpuInfo.basic_info[n] => description : 'Direct Rendering'

    Log:
    gpu_data.gpuInfo.basic_info[n] => description : GL_VERSION
    """

    def test_get_chrome_gpu_data(self):
        self.browser.get(CHROME_GPU_URL)
        config = load_configuration()
        gpu_js = 'window.setTimeout("browserBridge = new gpu.BrowserBridge();\
                 ",1000); return browserBridge'
        self.chrome_gpu_data = self.browser.execute_script(gpu_js)

        try:
            chrome_version = config['chrome']['version']
        except ConfigException, e:
            print "%s => you must provide chrome version in config.json" % e
            print "e.g. config['chrome']['version'] == 'Chrome/30'"

        config_chrome_version = str(self.chrome_gpu_data['clientInfo']['version'].split('.')[0])
        direct_rendering_enabled = filter_list_of_dicts(self.chrome_gpu_data['gpuInfo']['basic_info'], 'Direct rendering', 'Yes')

        assert (direct_rendering_enabled == True)
        assert (chrome_version == config_chrome_version)
