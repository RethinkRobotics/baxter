#!/bin/bash

tf=$(tempfile)
trap "rm -f -- '${tf}'" EXIT

topdir=$(dirname $(readlink -m ${0}))
master_uri=""

if [ -n "${1}" ]; then
	master_uri="${1}"
fi

cat <<-EOF > ${tf}
	if [ -s "\${HOME}"/.bashrc ]; then
		source "\${HOME}"/.bashrc
	fi

	if ! source /opt/ros/electric/setup.bash; then
		echo "Failed to source setup for ROS electric"
		exit 1
	fi

	export ROS_PACKAGE_PATH=${topdir}:\${ROS_PACKAGE_PATH}
	[[ -n "${master_uri}" ]] && export ROS_MASTER_URI="http://${master_uri}:11311"
EOF

${SHELL} --rcfile ${tf}

rm -f -- "${tf}"
trap - EXIT

# vim: noet
