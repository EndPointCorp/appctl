#!/usr/bin/env bash

set -e
set -x


cp -r /root/ssh /root/.ssh
chown -R root:root /root/.ssh
chmod 0600 /root/.ssh/id_rsa
eval $(ssh-agent -s)

ssh aptly@ln105.endpoint.com mkdir -p incoming/appctl/master/
cd /catkin_ws
scp *.dev aptly@${APTLY_SERVER}:incoming/appctl/master/
ssh aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming.sh --project appctl --branch master --rosrel "melodic" --distro "bionic"
ssh aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming-seperate-repos.sh --project appctl --branch master --rosrel "melodic" --distro "bionic"
