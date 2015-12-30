1.2.0 (2015-12-21)
---------------------------------
- No changes other than version bumps

1.1.1 (2015-4-15)
---------------------------------
- Updates baxter.sh to use a cross platform compatible command for making temporary directories

1.1.0 (2015-1-2)
---------------------------------
- Updates baxter.sh to default to indigo ROS version

1.0.0 (2014-5-1)
---------------------------------
- Updates rosinstall to use https repo checkouts (no SSH keys required)
- Updates baxter.sh to support 'sim' (local) environment settings argument
- Verbose baxter.sh run location and shell status

0.7.0 (2013-11-21)
---------------------------------
- The baxter repository replaces sdk-examples. This package contains metapackages and files for installation of the 'broken out' sdk repository structure.
- Reorganization of the sdk-examples repository into: baxter, baxter_interface, baxter_tools, baxter_examples, and baxter_common repositories. Catkin/Bloom compliance.
- Creation of baxter_sdk metapackage. This metapackage contains all respositories required for Baxter SDK usage.
- Catkinization of the SDK. Rosbuild no longer supported.
- Adds baxter.sh convenience script for Catkin environment setup.
- Adds baxter_sdk.rosinstall file for easy installation of all necessary SDK repositories.
