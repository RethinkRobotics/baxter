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

tf=$(tempfile)
trap "rm -f -- '${tf}'" EXIT

topdir=$(dirname $(readlink -m ${0}))
master_uri=""
ipaddr=""

if [ -n "${1}" ]; then
	master_uri="${1}"
fi

cat <<-EOF > ${tf}
	if [ -s "\${HOME}"/.bashrc ]; then
		source "\${HOME}"/.bashrc
	fi

	if [ -s "\${HOME}"/.bash_profile ]; then
		source "\${HOME}"/.bash_profile
	fi

	eval version=${2}
	if [ -z "\${version}" ]; then
		auto=$(ls -1 /opt/ros | tail -n 1)
		if [ -z "\${auto}" ]; then
			echo "No ROS installation found in /opt/ros/"
			exit 1
		fi
		ros_setup="/opt/ros/\${auto}"
	elif [ "\${version:0:1}" == "/" ]; then
		ros_setup="\${version}"
	else
		ros_setup="/opt/ros/\${version}"
	fi

	if [ ! -s "\${ros_setup}"/setup.sh ]; then
		echo "Failed to find the ROS environment script: "\${ros_setup}"/setup.bash"
		exit 1
	fi

	source \${ros_setup}/setup.bash 2>/dev/null || source \${ros_setup}/setup.sh

	export ROS_PACKAGE_PATH=${topdir}:\${ROS_PACKAGE_PATH}
	[[ -n "${master_uri}" ]] && export ROS_MASTER_URI="http://${master_uri}:11311"
	[[ -n "${ipaddr}" ]] && export ROS_IP="${ipaddr}"

	# setup the bash prompt
	export __ROS_PROMPT=\${__ROS_PROMPT:-0}
	if [ \${__ROS_PROMPT} -eq 0 -a -n "\${PROMPT_COMMAND}" ]; then
		export __ORIG_PROMPT_COMMAND=\${PROMPT_COMMAND}
	fi
	__ros_prompt () {
		if [ -n "\${__ORIG_PROMPT_COMMAND}" ]; then
			eval \${__ORIG_PROMPT_COMMAND}
		fi
		if ! echo \${PS1} | grep 'ros -' &>/dev/null; then
			export PS1="\[\033[00;33m\][ros - \${ROS_MASTER_URI}]\[\033[00m\] \${PS1}"
		fi
	}

	if [ "\${TERM}" != "dumb" ]; then
		export PROMPT_COMMAND=__ros_prompt
		__ROS_PROMPT=1
	elif ! echo \${PS1} | grep 'ros -' &>/dev/null; then
		export PS1="[ros - \${ROS_MASTER_URI}] \${PS1}"
	fi
EOF

${SHELL} --rcfile ${tf}

rm -f -- "${tf}"
trap - EXIT

# vim: noet
