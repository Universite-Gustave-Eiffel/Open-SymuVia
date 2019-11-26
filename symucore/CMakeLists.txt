cmake_minimum_required (VERSION 3.3)
project (SymuCore)

# Test 32/64 bits
if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    message(STATUS "Target is 64 bits")
    set(IS_X64 "True")
else("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    message(STATUS "Target is 32 bits")
    set(IS_X64 "False")
endif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Activate C++11 support under Linux
if(CMAKE_COMPILER_IS_GNUCXX OR APPLE)
    list( APPEND CMAKE_CXX_FLAGS "-std=c++0x")
endif()
 
if(MSVC)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996")
   set(CMAKE_CONFIGURATION_TYPES Debug Release )
   set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
     "Reset the configurations to what we need"
     FORCE)
   SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zc:wchar_t-")
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zc:wchar_t-")
 endif(MSVC)
 
 include_directories(${CMAKE_SOURCE_DIR})
 
# Headers Boost
if(MSVC)
get_filename_component(SDK_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../SDKs" REALPATH)
set(BOOST_INCLUDE_DIRECTORY ${SDK_DIRECTORY}/boost/boost_1_70_0 CACHE PATH "Boost headers folder")
include_directories(
                    $<$<CONFIG:Debug>:${BOOST_INCLUDE_DIRECTORY}>
                    $<$<CONFIG:Release>:${BOOST_INCLUDE_DIRECTORY}>)
else(MSVC)
set(BOOST_INCLUDE_DIRECTORY "/usr/local/include" CACHE PATH "boost headers folder")
include_directories(${BOOST_INCLUDE_DIRECTORY}) 
endif(MSVC)

# Set processor flags
add_definitions(-DSYMUCORE_EXPORTS)

file(GLOB SymuCore_src
    "Graph/Model/Cost.cpp"
    "Graph/Model/Cost.h"
    "Graph/Model/Penalty.cpp"
    "Graph/Model/Penalty.h"
    "Graph/Model/Graph.cpp"
    "Graph/Model/Graph.h"
    "Graph/Model/Link.cpp"
    "Graph/Model/Link.h"
    "Graph/Model/ListTimeFrame.cpp"
    "Graph/Model/ListTimeFrame.h"
    "Graph/Model/MultiLayersGraph.cpp"
    "Graph/Model/MultiLayersGraph.h"
    "Graph/Model/Node.cpp"
    "Graph/Model/Node.h"
    "Graph/Model/NullPattern.cpp"
    "Graph/Model/NullPattern.h"
    "Graph/Model/NullPenalty.cpp"
    "Graph/Model/NullPenalty.h"
    "Graph/Model/Pattern.cpp"
    "Graph/Model/Pattern.h"
    "Graph/Model/PatternsSwitch.cpp"
    "Graph/Model/PatternsSwitch.h"
    "Graph/Model/Driving/DrivingPattern.cpp"
    "Graph/Model/Driving/DrivingPattern.h"
    "Graph/Model/Driving/DrivingPenalty.cpp"
    "Graph/Model/Driving/DrivingPenalty.h"
    "Graph/Model/PublicTransport/PublicTransportLine.cpp"
    "Graph/Model/PublicTransport/PublicTransportLine.h"
    "Graph/Model/PublicTransport/PublicTransportPattern.cpp"
    "Graph/Model/PublicTransport/PublicTransportPattern.h"
	"Graph/Model/PublicTransport/WalkingPattern.cpp"
    "Graph/Model/PublicTransport/WalkingPattern.h"
    "Graph/Model/AbstractPenalty.cpp"
    "Graph/Model/AbstractPenalty.h"
    "Graph/Algorithms/Dijkstra.cpp"
    "Graph/Algorithms/Dijkstra.h"
    "Graph/Algorithms/KShortestPaths.cpp"
    "Graph/Algorithms/KShortestPaths.h"
    "Graph/Algorithms/ShortestPathsComputer.cpp"
    "Graph/Algorithms/ShortestPathsComputer.h"
    "Demand/DefaultMacroType.cpp"
    "Demand/DefaultMacroType.h"
    "Demand/DefaultPopulation.cpp"
    "Demand/DefaultPopulation.h"
    "Demand/Destination.cpp"
    "Demand/Destination.h"
    "Demand/MacroType.cpp"
    "Demand/MacroType.h"
    "Demand/Motive.cpp"
    "Demand/Motive.h"
    "Demand/Origin.cpp"
    "Demand/Origin.h"
    "Demand/Path.cpp"
    "Demand/Path.h"
    "Demand/Population.cpp"
    "Demand/Population.h"
    "Demand/Populations.cpp"
    "Demand/Populations.h"
	"Demand/PredefinedPath.cpp"
    "Demand/PredefinedPath.h"
    "Demand/Trip.cpp"
    "Demand/Trip.h"
    "Demand/SubPopulation.cpp"
    "Demand/SubPopulation.h"
    "Demand/ValuetedPath.cpp"
    "Demand/ValuetedPath.h"
    "Demand/VehicleType.cpp"
    "Demand/VehicleType.h" 
    "Utils/Point.cpp"
    "Utils/Point.h"
    "Utils/SymuCoreConstants.cpp"
    "Utils/SymuCoreConstants.h"
	"Utils/TravelTimeUtils.cpp"
	"Utils/TravelTimeUtils.h"
	"Utils/SpatialMarginalsHelper.cpp"
	"Utils/SpatialMarginalsHelper.h"
	"Users/IUserHandler.cpp"
	"Users/IUserHandler.h"
    "SymuCoreExports.h"
 )
 
# Define SymuCore libraries
add_library(SymuCore SHARED ${SymuCore_src})


# Define libraries to link

# Boost
if(IS_X64)
    if(MSVC)
        set(BOOST_LIB_DIRECTORY ${SDK_DIRECTORY}/boost/boost_1_70_0/lib/x64/ CACHE PATH "Boot x64 libraries folder")
    else(MSVC)
        set(BOOST_LIB_DIRECTORY "/usr/local/lib" CACHE PATH "Boost libs folder (x64)")
    endif(MSVC)
else(IS_X64)
set(BOOST_LIB_DIRECTORY ${SDK_DIRECTORY}/boost/boost_1_70_0/lib/win32/ CACHE PATH "Boost x32 libs folder")
endif(IS_X64)

if(MSVC)
set(BOOST_DATE_TIME_RELEASE_LIB_NAME "libboost_date_time-vc141-mt-x64-1_70.lib" CACHE STRING "Boost date_time release library name")
set(BOOST_DATE_TIME_DEBUG_LIB_NAME "libboost_date_time-vc141-mt-gd-x64-1_70.lib" CACHE STRING "Boost date_time debug library name")
elseif(APPLE)
set(BOOST_DATE_TIME_RELEASE_LIB_NAME "libboost_date_time.dylib" CACHE STRING "Boost date_time release library name")
set(BOOST_LOCALE_RELEASE_LIB_NAME "libboost_locale.dylib" CACHE STRING "Boost date_time debug library name")
else()
set(BOOST_DATE_TIME_RELEASE_LIB_NAME "libboost_date_time.so" CACHE STRING "Boost date_time release library name")
set(BOOST_LOCALE_RELEASE_LIB_NAME "libboost_locale.so" CACHE STRING "Boost date_time debug library name")
endif()


if(MSVC)
target_link_libraries( SymuCore 
                        $<$<CONFIG:Debug>:${BOOST_LIB_DIRECTORY}/${BOOST_DATE_TIME_DEBUG_LIB_NAME}>
                        $<$<CONFIG:Release>:${BOOST_LIB_DIRECTORY}/${BOOST_DATE_TIME_RELEASE_LIB_NAME}> )
else(MSVC)
target_link_libraries( SymuCore ${BOOST_LIB_DIRECTORY}/${BOOST_DATE_TIME_RELEASE_LIB_NAME}
                               ${BOOST_LIB_DIRECTORY}/${BOOST_LOCALE_RELEASE_LIB_NAME})
endif(MSVC)
      
# Define output directory
set_target_properties( SymuCore
	    PROPERTIES 
		ARCHIVE_OUTPUT_DIRECTORY  "${CMAKE_SOURCE_DIR}/../../bin"
		LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_SOURCE_DIR}/../../bin"
		RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_SOURCE_DIR}/../../bin"
		
		# Prevent creation of Debug/Reelase folders under bin (MSVC)  
		ARCHIVE_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_SOURCE_DIR}/../../bin"
		LIBRARY_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_SOURCE_DIR}/../../bin"
		RUNTIME_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_SOURCE_DIR}/../../bin"
		ARCHIVE_OUTPUT_DIRECTORY_DEBUG  "${CMAKE_SOURCE_DIR}/../../bin"
		LIBRARY_OUTPUT_DIRECTORY_DEBUG  "${CMAKE_SOURCE_DIR}/../../bin"
		RUNTIME_OUTPUT_DIRECTORY_DEBUG  "${CMAKE_SOURCE_DIR}/../../bin"
		)
		
# Define output name ended with "d" for debug
set_target_properties( SymuCore
    PROPERTIES
    OUTPUT_NAME_DEBUG "SymuCored"
)

