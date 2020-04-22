#!/usr/bin/env bash

set -e
set -x


cp -r /root/ssh /root/.ssh
chown -R root:root /root/.ssh
chmod 0600 /root/.ssh/id_rsa
eval $(ssh-agent -s)

ssh -o StrictHostKeychecking=no aptly@${APTLY_SERVER} mkdir -p incoming/appctl/master/
cd /catkin_ws
scp -o StrictHostKeychecking=no *.deb aptly@${APTLY_SERVER}:incoming/appctl/master/
ssh -o StrictHostKeychecking=no  aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming.sh --project appctl --branch master --rosrel "melodic" --distro "bionic"
ssh -o StrictHostKeychecking=no aptly@${APTLY_SERVER} bash /home/aptly/bin/publish-incoming-seperate-repos.sh --project appctl --branch master --rosrel "melodic" --distro "bionic"
