#!/bin/bash

# This script should idempotently regenerate and import the ssl certificate to nssdb
NSSDB=${HOME}/.pki/nssdb
imported=`certutil -d sql:$NSSDB -L | grep -c rosbridge`
force=${1:-0}

if [[ $imported -ge 1 && $force != "force" ]] ; then
  echo "SSL key is already imported to sql:$NSSDB"
  echo "If you want force-regenerate it, run this script with force: '`basename $0` force'"
else
  echo "Generating ros.crt and key"
  /usr/bin/openssl req -nodes -new -x509 -keyout ros.key -out ros.crt -days 3650 -config catkin/src/portal/launch/ssl/ros-openssl.cfg
  if [ -d $NSSDB ] ; then
    echo "nssdb file exists. carry on..."
  else
    echo "nssdb database was not initialized - initializing.."
    certutil -d sql:$NSSDB -N --empty-password
  fi
  certutil -d sql:$NSSDB -A -t "P,," -n rosbridge -i ros.crt
fi
