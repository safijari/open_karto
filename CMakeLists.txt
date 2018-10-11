cmake_minimum_required(VERSION 2.8.3)
project(open_karto)
find_package(Boost REQUIRED COMPONENTS thread)

add_subdirectory(pybind11)

include_directories(include /usr/include/eigen3/)

add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)

pybind11_add_module(openkarto src/PythonInterface.cpp)

target_link_libraries(openkarto ${Boost_LIBRARIES} karto blas lapack cholmod cxsparse)

INSTALL(CODE "
    include(BundleUtilities)
    fixup_bundle(\"openkarto\" \"/usr/local/lib\")
    " COMPONENT Runtime)


# To Create a package, one can run "cpack -G DragNDrop CPackConfig.cmake" on Mac OS X
# where CPackConfig.cmake is created by including CPack
# And then there's ways to customize this as well
set(CPACK_BINARY_DRAGNDROP ON)
include(CPack)
