echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
# make -j
make -j 2>&1 | grep error
