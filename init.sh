#!/bin/bash

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is to be used in the *root* of your Catkin workspace.

# This is a convenient script which will set up your ROS environment and
# should be sourced with every new instance of a shell in which you plan on
# working with Baxter.

#--------------------------------------------------------------------------#
#               USER CONFIGURABLE ROS ENVIRONMENT VARIABLES                #
#--------------------------------------------------------------------------#
# Specify Baxter's hostname
baxter_hostname="baxter.local"

# Set *Either* your ros_ip or ros_hostname. Please note if using ros_hostname
# that this must be resolvable to Baxter.
ros_ip="192.168.XXX.XXX"
#ros_hostname="my_computer.local"

# Specify ROS distribution (e.g. groovy, hydro)
ros_version="groovy"
#--------------------------------------------------------------------------#

tf=$(tempfile)
trap "rm -f -- '${tf}'" EXIT

topdir=$(dirname $(readlink -m ${0}))

cat <<-EOF > ${tf}
	if [ -s "\${HOME}"/.bashrc ]; then
		source "\${HOME}"/.bashrc
	fi

	if [ -s "\${HOME}"/.bash_profile ]; then
		source "\${HOME}"/.bash_profile
	fi

	# verify ros_version lowercase
	ros_version="$(tr [A-Z] [a-z] <<< "${ros_version}")"

	if [ ! -d "/opt/ros" ] || [ ! "$(ls -A /opt/ros)" ]; then
		echo "EXITING - No ROS ros installation found in /opt/ros."
		echo "Is ROS installed?"
		exit 1
	fi

	if [ -n ${ros_ip} ] && [[ "${ros_ip}" == "192.168.XXX.XXX" ]]; then
		echo -ne "EXITING - Please edit this file, modifying the ros_ip to \
reflect your current IP address.\n"
		exit 1
	fi

	if [ -n "${ros_ip}" ] && [ -n "${ros_hostname}" ]; then
		echo -ne "EXITING - Please edit this file, modifying to specify \
*EITHER* ros_ip or ros_hostname.\n"
		exit 1
	fi

	if [ ! -n "${ros_ip}" ] && [ ! -n "${ros_hostname}" ]; then
		echo -ne "EXITING - Please edit this file, modifying to specify your \
ros_ip or ros_hostname.\n"
		exit 1	
	fi
	ros_setup="/opt/ros/\${ros_version}"

	if [ ! -d "\${ros_setup}" ]; then
		echo -ne "EXITING - Failed to find ROS \${ros_version} installation at \
\${ros_setup}.\n"
		exit 1
	fi

	if [ ! -s "\${ros_setup}"/setup.sh ]; then
		echo -ne "EXITING - Failed to find the ROS environment script: \
"\${ros_setup}"/setup.sh.\n"
		exit 1
	fi

	if [ ! -s "devel/setup.bash" ]; then
		echo -ne "EXITING - Please verify that this script is being run in the \
root of your catkin workspace, and that your workspace has been built \
(catkin_make).\nSource this script again upon completion of your workspace \
build.\n"
		exit 1
	fi

	source \${ros_setup}/setup.bash 2>/dev/null || source \${ros_setup}/setup.sh

	[ -n "${ros_ip}" ] && export ROS_IP="${ros_ip}"
	[ -n "${ros_hostname}" ] && export ROS_HOSTNAME="${ros_hostname}"

    export ROS_MASTER_URI="http://${baxter_hostname}:11311"

	source devel/setup.bash

	# setup the bash prompt
	export __ROS_PROMPT=\${__ROS_PROMPT:-0}
	if [ \${__ROS_PROMPT} -eq 0 -a -n "\${PROMPT_COMMAND}" ]; then
		export __ORIG_PROMPT_COMMAND=\${PROMPT_COMMAND}
	fi

	__ros_prompt () {
		if [ -n "\${__ORIG_PROMPT_COMMAND}" ]; then
			eval \${__ORIG_PROMPT_COMMAND}
		fi
		if ! echo \${PS1} | grep 'ros' &>/dev/null; then
			export PS1="\[\033[00;33m\][ros \${ros_version} - \
\${ROS_MASTER_URI}]\[\033[00m\] \${PS1}"
		fi
	}

	if [ "\${TERM}" != "dumb" ]; then
		export PROMPT_COMMAND=__ros_prompt
		__ROS_PROMPT=1
	elif ! echo \${PS1} | grep 'ros' &>/dev/null; then
		export PS1="[ros \${ros_version} - \${ROS_MASTER_URI}] \${PS1}"
	fi

EOF

${SHELL} --rcfile ${tf}

rm -f -- "${tf}"
trap - EXIT

# vim: noet
