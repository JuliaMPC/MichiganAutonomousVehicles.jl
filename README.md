# MAVs


## Installation

In Julia, you can install the MAVs.jl package by typing:
```julia
Pkg.clone("https://github.com/huckl3b3rry87/MAVs.jl")
```


## LiDAR Model

make sure that you are in the build folder:
```terminal
febbo@febbo-HP-Z220-SFF-Workstation:~/.julia/v0.5/MAVs/workspace/build/lidar_model$ gazebo --verbose /home/febbo/.julia/v0.5/MAVs/workspace/src/lidar_model/velodyne.world
```


To dymically control the LiDAR:

http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

febbo@febbo-HP-Z220-SFF-Workstation:~/.julia/v0.5/MAVs/workspace/devel/lib/lidar_model$ ls
vel
