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