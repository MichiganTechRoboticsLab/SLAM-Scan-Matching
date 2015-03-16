# project
cmake_minimum_required (VERSION 2.6)
project (libicp)

# directories
set (LIBICP_SRC_DIR src)
FIND_PATH(BOOST_DIR "boost")

# include directory
include_directories("${LIBICP_SRC_DIR}")
include_directories("${BOOST_DIR}")

# sources
FILE(GLOB LIBICP_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# build demo program
add_executable(icp ${LIBICP_SRC_FILES})

