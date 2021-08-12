cmake_minimum_required(VERSION 3.10)

project(BatteringRam)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} include)

add_executable( BatteringRam src/Vehicle.cpp
                             src/Trajectory.cpp
							 src/RSC.cpp
							 src/RamTree.cpp
							 src/Map.cpp
							 src/main.cpp
							 src/Immovable.cpp
							 src/brutil.cpp
							 src/Blobstacle.cpp
							 src/AbstractTrajectory.cpp)

target_link_libraries( BatteringRam ${OpenCV_LIBS} )