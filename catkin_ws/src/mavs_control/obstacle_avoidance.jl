#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle
@rosimport mavs_control.msg: Control

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg
using obstacle_detector.msg
using mavs_control.msg

using NLOptControl
using VehicleModels
using DataFrames
using MAVs
using KNITRO

# TODO
# 1) detect crash
# 3) get the Gazebo sim time
# 5) pass these parameters to a service or something -> needs to happen on a seperate node!
# 6) update OCP based off of current gazebo model position
# 7) get the current position of Gazebo model => need all of the states!!
# 8) get time from the Gazebo world

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

  Q = getvalue(params[2][7]); # number of obstacles the algorithm can handle
  L = length(r)               # number of obstacles detected
  N = Q - L;
  if N < 0
    warn(" \n The number of obstacles detected exceeds the number of obstacles the algorithm was designed for! \n
              Consider increasing the number of obstacles the algorithm can handle \n!")
  end

  for i in 1:Q
    if i <= L          # add obstacle
      setvalue(params[2][1][i],r[i]);
      setvalue(params[2][2][i],r[i]);
      setvalue(params[2][3][i],x[i]);
      setvalue(params[2][4][i],y[i]);
      setvalue(params[2][5][i],vx[i]);
      setvalue(params[2][6][i],vy[i]);
    else              # set obstacle off field
      setvalue(params[2][1][:],1.0);
      setvalue(params[2][2][:],1.0);
      setvalue(params[2][3][:],-100.0);
      setvalue(params[2][4][:],-100.0);
      setvalue(params[2][5][:],0.0);
      setvalue(params[2][6][:],0.0);
    end
  end

  return nothing
end

function sendOptData(n)

  X0=n.X0
  t=n.r.t_ctr+n.mpc.t0
  U=n.r.U
  t0=n.mpc.t0_actual
  tf=n.r.eval_num*n.mpc.tex

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

  # message for solution to optimal control problem
  const pub = Publisher{Control}("/mavs/optimal_control", queue_size=10)

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
    updateAutoParams!(n,c);  # check to see if goal is in range

    # get the current position of Gazebo model
    gs = GetModelStateRequest()
    gs.model_name = modelName
    gs_r = get_state(gs)
    if !gs_r.success
        error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
    end
    X0 = zeros(8)
    X0[1] = gs_r.pose.position.x
    X0[2] = gs_r.pose.position.y
    X0[3] = X0p[3]
    X0[4] = X0p[4]
    X0[5] = gs_r.pose.orientation.z
    X0[6] = X0p[6]
    X0[7] = X0p[7]
    X0[8] = X0p[8]
    updateX0!(n,X0;(:userUpdate=>true));
    status=autonomousControl!(n);                # rerun optimization
    # NOTE not sure what this is doing currently
    n.mpc.t0_actual=(n.r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
    msg = Control()
    msg.t = n.r.t_st
    msg.x = n.r.X[:,1]
    msg.y = n.r.X[:,2]
    msg.psi = n.r.X[:,5]
    msg.sa = n.r.X[:,6]
    msg.vx = n.r.X[:,7]
    publish(pub,msg)

    rossleep(loop_rate)
  end
end

if !isinteractive()
    main()
end
