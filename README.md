# Portal project

## Overview

Portal is a set or ROS nodes that provide communication layer for Chrome browsers running various extensions.

## Details

### Directory Structure


|-- bin                  - PLACE FOR SCRIPTS
|  L-- run_tests        - SCRIPT FOR RUNNING ALL TESTS
|-- extensions           - PLACE EXTENSION DIRECTORIES HERE
|  |-- crxmake          - TOOL FOR PACKING CRX FILES
|  |-- sigs             - DIRECTORY FOR TEMPORARY FILES
|  |-- <extensionA>
|  |--  ...             - PACKAGED EXTENSION FILES (.crx,.pub,.eid,-upd.xml) ARE DROPPED ALONG-SIDE
|  L-- <extensionZ>
|-- keys
|  |-- README
|  |-- <extensionA.pem> - KEYS ARE NOT COMMITED TO GIT
|  |--  ...
|  L-- <extensionZ.pem>
|-- managed_policy_example
|  |-- <policy_exampleA.json>
|  |--  ...
|  L-- <policy_exampleZ.json>
|-- tests                         - TESTS FOR THE PROJECT
|  |-- extensions                - TESTS FOR EXTENSIONS
|   |  L-- functional            - FUNCTIONAL TESTS USING SELENIUM
|   |      |-- requirements.txt  - FILE FOR INSTALLING THE REQUIREMENTS WITH PIP
|   |      L-- tests
|   |          |-- ...           - TEST FILES
|   |          L-- <test_z>
|  L-- README           - README FILE FOR TESTS
|-- check-extensions     - USE THIS TO VERIFY ALL EXTENSIONS
|-- doc-extensions       - USE THIS TO JSDOC ALL EXTENSIONS
|-- pack-extensions      - USE THIS TO RE-PACKAGE ALL EXTENSIONS
|-- publish-extensions   - USE THIS TO PUBLISH ALL EXTENSIONS
└-- README               - THIS README

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

