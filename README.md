# depthai-ros-examples
noetic-devel branch also works on melodic(tested). Might also work on kinetic too.


## Getting Started
### Setting up procedure

1. `cd ~`
2. `git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop`
3. `cd ~/depthai-core`
4. `mkdir build`
5. `cd build`
6. `cmake .. -D BUILD_SHARED_LIBS=ON`
7. `cmake --build . --parallel --config Release --target install`   
8. `cd ~`
9. `mkdir -p ros_ws/src`
10. `cd ros_ws/src`
11. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
12. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
13. `cd ~/ros_ws`
14. `source /opt/ros/<ros-distro>/setup.zsh`     
15. `catkin_make_isolated --cmake-args -D depthai_DIR=${depthai-core insall directory}/lib/cmake/depthai`
