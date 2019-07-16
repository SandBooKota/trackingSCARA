cmake_minimum_required(VERSION 3.1.0)

project(trackingSCARA)

find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

add_executable(trackingSCARA main.cpp arduino-serial-lib.cpp cv-helpers.hpp)
set_property(TARGET trackingSCARA PROPERTY CXX_STANDARD 11)
target_link_libraries(trackingSCARA m ${DEPENDENCIES})

set_target_properties (trackingSCARA PROPERTIES
	FOLDER "Examples/OpenCV"
)

install(
	TARGETS

	trackingSCARA

	RUNTIME DESTINATION
	${CMAKE_INSTALL_PREFIX}/bin
)
