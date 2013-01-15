#!/bin/bash

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

	if [ ! -s /opt/ros/electric/setup.bash -a ! -s /opt/ros/electric/setup.sh ]; then
		echo "Failed to find environment script for ROS electric"
		exit 1
	fi

	source /opt/ros/electric/setup.bash 2>/dev/null || source /opt/ros/electric/setup.sh

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
