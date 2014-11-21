### Installation


#### Python Packages

```
pip install --user -r tests/requirements.txt
```

#### Chrome

```
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
sudo sh -c 'echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
```


#### Xfvb

``` 
sudo apt-get install xvfb
```

For the headless tests we need to have the xvfb running. This can be achieved with a command: 

```
sudo Xvfb :10 -ac
```

### Running tests

The test suite is configured via a JSON configuration file
See config-example.json template for guidance
when creating one.
The valid config.json can live anywhere, the env
variable PORTAL_TESTS_CONFIG need to point to it,
the test suite fails otherwise.

Run the tests from the main project directory:

```
py.test
py.test tests/extensions/functional/ to speed up discovery
py.test -s to capture stdout
py.test -k test_url_change_after_search to run only 1 particular test case
```


### ROS enabled tests (developer version)

Requirements:

- proper config.json with multiple chrome instances

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-ros-base ros-indigo-rosbridge-server ros-indigo-geographic-msgs
```

- modify /etc/hosts so 42-b is pointing to localhost
- manage SSL for secure websocket connection
- install package that provides certutil
```
sudo apt-get install libnss3-tools
```
- generate the ssl key and import it to browser's nssdb
```
./catkin/src/portal/launch/bin/manage_ssl.sh
```

- add following line to ~/.bashrc for convenience
```
if [ -f /opt/ros/indigo/setup.bash ] ; then
    . /opt/ros/indigo/setup.bash
fi
```

>>>> For the above lines I get: 
λ ymon portal → λ git topic/selenium* → source /opt/ros/indigo/setup.bash 
/opt/ros/indigo/setup.bash:.:8: no such file or directory: /home/szymon/projects/ep/portal/setup.sh
>>>>

- following commands are run just once or if something changes in the catkin workspace
```
cd catkin
catkin_make
```


#### Test Scenario:

Build and launch ros nodes. Everything run from the project directory.

Run in one terminal:

```
source catkin/devel/setup.bash
roslaunch catkin/src/portal/launch/portal.launch
```

Run the tests in another terminal (PORTAL_TESTS_CONFIG has to be set):

```
source catkin/devel/setup.bash
py.test -s -k test_ros_position_after_search
```


#### TODO - discuss

- make asserts on whether specific topics/publishers/subscribers exist
- make other ros specific basic asserts
- make sure that browsers are connected (selenium wss or ros object is connected to a wss socket/port/whatever)
- perform browsers actions that result in ROS traffic and make asserts on that traffic
- make some google/tactile/portal specific scenarios tests



### ROS enabled tests (ci box version)

raise NotImplementedException
