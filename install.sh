# add a few things to your setup
echo '. /opt/ros/cturtle/setup.sh' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=/home/ils/ros:${ROS_PACKAGE_PATH}' >> ~/.bashrc

# need svn
sudo apt-get install subversion

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

# make needed packages
cd ~/ros/brown-ros-pkg/teleop_twist_keyboard; make
cd ~/ros/brown-ros-pkg/irobot_create_2_1; make
cd ~/ros/vanadium-ros-pkg/vanadium_drivers/arbotix; make
cd ~/ros/bosch-ros-pkg/usb_cam; make
