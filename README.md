# Robotics C++ code collection
**Work in progress**: Simple collection of C++ codes concerning robotics (mainly autonomous and mobile robotics algorithms).

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

## Implemented algorithms

#### Planning -> RRT (Rapidly-exploring Random Tree)

![](media/rrt_path.gif)

![](media/rrt_no_path.gif)

#### Other -> ICP (Iterative Closest Point, SVD-based, 2D)

![](media/icp_svd.gif)

<!-- ### Planning -->
<!-- ### Localization -->
<!-- ### SLAM -->
<!-- ### Other -->
