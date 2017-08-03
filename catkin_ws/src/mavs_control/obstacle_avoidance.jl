#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg
using obstacle_detector.msg

using NLOptControl
using VehicleModels
using DataFrames
using MAVs
using KNITRO

# TODO
# 1) detect crash
# 3) get the Gazebo sim time
# 4) pause Gazebo at the start until everything is ready
# 5) pass these parameters to a service or something -> needs to happen on a seperate node!
# 6) update OCP based off of current gazebo model position


"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 8/2/2017 \n
--------------------------------------------------------------------------------------\n
"""
function setObstacleData(params)

  r=deepcopy(RobotOS.get_param("obstacle_radius"))
  x=deepcopy(RobotOS.get_param("obstacle_x"))
  y=deepcopy(RobotOS.get_param("obstacle_y"))
  vx=deepcopy(RobotOS.get_param("obstacle_vx"))
  vy=deepcopy(RobotOS.get_param("obstacle_vy"))

  if r!=NaN
    # update obstacle feild
  #  if length(r)==1 # single obstacle
  #    setvalue(params[2][1],r[1]);
  #    setvalue(params[2][2],r[1]);
  #    setvalue(params[2][3],x[1]);
  #    setvalue(params[2][4],y[1]);
  #    setvalue(params[2][5],vx[1]);
  #    setvalue(params[2][6],vy[1]);
  #  else
      for i in 1:length(r)
        setvalue(params[2][1][i],r[i]);
        setvalue(params[2][2][i],r[i]);
        setvalue(params[2][3][i],x[i]);
        setvalue(params[2][4][i],y[i]);
        setvalue(params[2][5][i],vx[i]);
        setvalue(params[2][6][i],vy[i]);
      end
  #  end
  else # setting the obstacles off the field
    setvalue(params[2][1][:],1.0);
    setvalue(params[2][2][:],1.0);
    setvalue(params[2][3][:],-100.0);
    setvalue(params[2][4][:],-100.0);
    setvalue(params[2][5][:],0.0);
    setvalue(params[2][6][:],0.0);
  end

  return nothing
end

# TODO make an inner loop that runs faster and sets the position of the robot

function main()
  init_node("rosjl_obstacles")

  ###############################
  # set up services and messages

  # unpause simulation
  const unpause_physics = ServiceProxy("/gazebo/unpause_physics",Empty)
  println("Waiting for '/gazebo/unpause_physics' service...")
  wait_for_service("/gazebo/unpause_physics")

  # Set up service to get Gazebo model state
  const get_state = ServiceProxy("/gazebo/get_model_state",GetModelState)
  println("Waiting for 'gazebo/get_model_state' service...")
  wait_for_service("gazebo/get_model_state")

  # set up services and messages
  ###############################

  c=defineCase(;(:mode=>:autoGazebo));
  n=initializeAutonomousControl(c);

  driveStraight!(n)

  loop_rate = Rate(2.0)
  # modelName = "robot"

  # unpause physics
  up = EmptyRequest()
  unpause_physics(up)

  while ! is_shutdown()
    println("Running model for the: ",n.r.eval_num," time");
    setObstacleData(n.params)
    updateAutoParams!(n,c);                      # update model parameters
    status=autonomousControl!(n);                # rerun optimization
    n.mpc.t0_actual=(n.r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
    simPlant!(n)

    rossleep(loop_rate)
  end
end

if !isinteractive()
    main()
end
