cmake_minimum_required(VERSION 2.8.12)
project(open_karto)
# Note: The lines below allows a pip install pybind11 to be found by cmake.
#       Do not remove.
execute_process(COMMAND python -c
                "import pybind11_cmake; print(pybind11_cmake.__path__[0])"
                OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
include_directories(/usr/include/eigen3 include)
pybind11_add_module(open_karto src/Karto.cpp src/Mapper.cpp src/PythonInterface.cpp)
target_link_libraries(open_karto PRIVATE Boost::thread)
