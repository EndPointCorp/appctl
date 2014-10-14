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

Run from the main project directory:

bin/run_tests

or if we want to have the headless tests:

DISPLAY=:10 bin/run_tests
