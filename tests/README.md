INSTALLATION
=============

Python Packages
--------------

pip install --user -r tests/requirements.txt

Chrome
--------------

wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
sudo sh -c 'echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
sudo apt-get update
sudo apt-get install google-chrome-stable

Xfvb
--------------
 
sudo apt-get install xvfb

Things to run
==============

For the headless tests we need to have the xvfb running. This can be achieved with a command: 

sudo Xvfb :10 -ac

RUNNING TESTS
==============

The test suite is configured via a JSON configuration file.
See config-example.json template for guidance
when creating one.
The valid config.json can live anywhere, the env
variable PORTAL_TESTS_CONFIG need to point to it,
the test suite fails otherwise.

Run the tests from the main project directory:

py.test
py.test tests/extensions/functional/ to speed up discovery
py.test -s to capture stdout
py.test -k test_url_change_after_search to run only 1 particular test case

ROS enabled selenium tests vs. ordinary selenium tests
======================================================

* vagrant tests (developer PC version)
 * boxes: headnode, display, kiosk
 * requirements: a development box with min. 4Gigs of free RAM - min.
   1.5GB per displaynode and prolly about 1-2 gigs per heanode
* vagrant + physical box tests (e.g. Mountain View version)
 * boxes: physical headnode, physical display, vagrant kiosk
 * requirements: physical headnode, one display node with nvidia K5000,
   vagrant installed on headnode

### ROS enabled tests (developer/vagrant version)

Scenario:

* WE CAN SET UP ROS + BROWSERS ON ONE MACHINE 

* checkout lg_chef
* build your display nodes and headnode
  * first render the test/vagrant/Vagrantfile and test/scripts/run_tests.sh by executing a following command in lg_chef repo root directory:
    rake render_tests[<path to test json template>,chef_zero,<path to node definition>]

    ```
    rake render_tests[test/vagrant/json/portal-selenium-pure-vagrant.json,chef_zero,nodes/lg-head-portaltest.json]
    ```
  * then converge the boxes by 
  *
*

Requirements:
* Vagrant (http://www.vagrantup.com)
* VirtualBox:
 * echo 'deb http://download.virtualbox.org/virtualbox/debian trusty contrib' | sudo tee /etc/apt/sources.list.d/virtualbox.list
 * wget -q https://www.virtualbox.org/download/oracle_vbox.asc -O- | sudo apt-key add -
 * sudo apt-get update -q && sudo apt-get install virtualbox-4.3 dkms
* lg_chef.git checked out somewhere in the

### ROS TODO:

*
