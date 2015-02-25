# Portal project

## Overview

Portal is a set or ROS nodes that provide communication layer for Chrome browsers running various extensions.

## Details

### Extensions

In order to pack extensions you should execute

```bash
./pack-extensions
```

Packaged Extension Files:

* In "<repo>/extensions/", see the following result files:
  * <extension>.pub - the public key portion of the extension-specific keypair
  * <extension>.eid - the extension-specific Application ID which is derived from the public key.
  * <extension>.crx - the extension itself, installable in Chrome/Chromium.
  * <extension>-upd.xml - the auto-update xml file which should be served to Chrome/Chromium.

### Catkin Workspace (ROS nodes)

#### Installing from source

To initialize the Catkin workspace for ROS packages, run "catkin_init_workspace" in catkin/src/ .
You must have ROS installed.

#### Installing from apt-get

You need to add a repo to /etc/apt/sources.list.d/portapt.list

```
### THIS FILE IS AUTOMATICALLY CONFIGURED ###
# You may comment out this entry, but any other modifications may be lost.
deb [arch=amd64] http://portapt.galaxy.endpoint.com/trusty-master trusty main
deb [arch=amd64] http://portapt.galaxy.endpoint.com/trusty-staging trusty main
deb [arch=amd64] http://portapt.galaxy.endpoint.com/trusty-development trusty main
```

and whitelist yourself so galaxy.endpoint.com doesn't block you.

After that you need to add the gpg key that's used for signing the messages.

```
wget -qO - http://portapt.galaxy.endpoint.com/archive.key | sudo apt-key add -
```

