#!/bin/sh

USER_ID=`id -u`

if [ "$USER_ID" != "0" ]; then
	echo Please run this script as root
	exit
fi


modprobe -r ioaccess
make install
depmod
modprobe ioaccess


cp Linux/ioaccess.modules /etc/sysconfig/modules/ioaccess.modules
chmod +x /etc/sysconfig/modules/ioaccess.modules

