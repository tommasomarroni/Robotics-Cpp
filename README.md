# Robotics-Cpp
Just a collection of C++ codes concerning robotics (mainly autonomous and mobile robotics algorithms).

## Requirements

- CMake
- OpenCV
- Eigen (Get Eigen >= 3.3.7 and put Eigen headers folder in usr/local/include)

## Build

```
mkdir build
cd build
cmake ../
make -j 8
```

## Algorithms
### Planning
#### RRT

RRT: path found

![](media/rrt_path.gif)

RRT: path not found

![](media/rrt_no_path.gif)

### Localization
### SLAM
### Other
