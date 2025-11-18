cmake_minimum_required(VERSION 2.8.3)

project(prop_arm_gazebo)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir . meshes meshes/stl sdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
