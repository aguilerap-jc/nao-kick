cmake_minimum_required(VERSION 2.8)
project(kickSheldon)

find_package(qibuild)

option(BUMPER_IS_REMOTE "module is compiles as a remote module (ON or OFF)"ON)

set(_srcs
	main.cpp
	bumper.h
	bumper.cpp
)

if(BUMPER_IS_REMOTE)
	add_definitions(" -DBUMPER_IS_REMOTE")
	qi_create_bin(kickSheldon ${_srcs})
else()
	qi_create_lib(kickSheldon SHARED %{_srcs} SUBFOLDER naoqi)
endif()

qi_use_lib(kickSheldon ALCOMMON ALPROXIES)
qi_use_lib(kickSheldon OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_VIDEO)
