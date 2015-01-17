Portal Config Service
=====================

This service returns the contents of an http-accessed json configuration file.

### serve\_config.py

This script serves the configuration data.  On each request, the configuration is retrieved again.

##### params

* `~url` - URL to the configuration json file. (default: http://lg-head/portal/config.json).

##### services

* `/portal_config/query` : `portal_config/PortalConfig`

