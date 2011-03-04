# a place for packages
cd; mkdir ros; cd ros

# our stuff
svn co http://albany-ros-pkg.googlecode.com/svn/trunk/ albany-ros-pkg
svn co http://vanadium-ros-pkg.googlecode.com/svn/trunk/ vanadium-ros-pkg

# create related
cd ~/ros; mkdir brown-ros-pkg; cd brown-ros-pkg
svn co https://brown-ros-pkg.googlecode.com/svn/tags/brown-ros-pkg/teleop_twist_keyboard teleop_twist_keyboard
svn co https://brown-ros-pkg.googlecode.com/svn/tags/brown-ros-pkg/irobot_create_2_1 irobot_create_2_1

# usb_cam
cd ~/ros; mkdir bosch-ros-pkg; cd bosch-ros-pkg
svn co https://bosch-ros-pkg.svn.sourceforge.net/svnroot/bosch-ros-pkg/tags/stacks/bosch_drivers/bosch_drivers-0.1.0/ .

# add a few things to your setup
echo '. /opt/ros/diamondback/setup.sh' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=~/ros:${ROS_PACKAGE_PATH}' >> ~/.bashrc

echo 'You should now "source ~/.bashrc" and run install.sh'
