cmake_minimum_required(VERSION 2.8.3)
project(yaskawa_gp25_support)

find_package(catkin REQUIRED)

catkin_package()

install(
	DIRECTORY config launch meshes urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
