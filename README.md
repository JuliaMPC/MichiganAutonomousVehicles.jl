# MAVs


## Installation

In Julia, you can install the MAVs.jl package by typing:
```julia
Pkg.clone("https://github.com/huckl3b3rry87/MAVs.jl")
```


## LiDAR Model

This model comes from [here](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)


### Installation Instructions
This is assuming that ros and catkin are setup on your machine.

#### Compile the Model on your Machine

Overlay the workspace on top of your environment in the `~/MAVs/workspace/devel` folder and sourcing the setup file as:
```terminal
source setup.bash
```

Make sure that it worked:
```terminal
echo $ROS_PACKAGE_PATH
```

Which gives:
```terminal
/home/febbo/.julia/v0.6/MAVs/workspace/src:/opt/ros/kinetic/share
```

Instead of doing this every time you start a terminal add this to the `.bashrc` file in the `home` directory:
```terminal
source /home/febbo/.julia/v0.6/MAVs/workspace/devel/setup.bash
```

Now, you can do things like this:
```terminal
febbo@febbo-HP-ZBook-17-G2:~$ roscd lidar_model/
febbo@febbo-HP-ZBook-17-G2:~/.julia/v0.6/MAVs/workspace/src/lidar_model$
```

Go into the`~/MAVs/workspace$` directory and run:
```terminal
catkin_make
```

#### Running the Model
make sure that you are in the `~/MAVs/workspace/build/lidar_model` directory and run
```terminal
gazebo --verbose /home/febbo/.julia/v0.6/MAVs/workspace/src/lidar_model/velodyne.world
```
**NOTE:** make sure that the directory in the above command is updated!


#### Dynamically Controlling the LiDAR:
Go into the `~/MAVs/workspace/devel/lib/lidar_model` directory and type
```terminal
ls
vel
```
