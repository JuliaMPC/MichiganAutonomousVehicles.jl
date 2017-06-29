using MAVs
using NLOptControl
using VehicleModels
using Parameters
using DataFrames

using Base.Test

# Moving Obstacle Avoidance Tests
include("moving_obstacles.jl")
@test goal_in_lidar()
