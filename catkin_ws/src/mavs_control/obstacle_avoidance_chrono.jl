#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties, GetLinkState, SetLinkState
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle
@rosimport mavs_control.msg: Control
@rosimport ros_chrono_msgs.msg: veh_status

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg
using obstacle_detector.msg
using mavs_control.msg
using ros_chrono_msgs.msg

using NLOptControl
using VehicleModels
using DataFrames
using MAVs
using Ipopt
using PyCall
@pyimport tf.transformations as tf

function setObstacleData(params)

  if RobotOS.has_param("obstacle_radius")
    Q = params[2][7];           # number of obstacles the algorithm can handle

    if isnan(RobotOS.get_param("obstacle_radius"))
      for i in 1:Q
        setvalue(params[2][1][i],1.0);
        setvalue(params[2][2][i],1.0);
        setvalue(params[2][3][i],-100.0 - 3*i);
        setvalue(params[2][4][i],-100.0);
        setvalue(params[2][5][i],0.0);
        setvalue(params[2][6][i],0.0);
      end
    else
      r=deepcopy(RobotOS.get_param("obstacle_radius"))
      x=deepcopy(RobotOS.get_param("obstacle_x"))
      y=deepcopy(RobotOS.get_param("obstacle_y"))
      vx=deepcopy(RobotOS.get_param("obstacle_vx"))
      vy=deepcopy(RobotOS.get_param("obstacle_vy"))

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
          setvalue(params[2][1][i],1.0);
          setvalue(params[2][2][i],1.0);
          setvalue(params[2][3][i],-100.0 - 3*i);
          setvalue(params[2][4][i],-100.0);
          setvalue(params[2][5][i],0.0);
          setvalue(params[2][6][i],0.0);
        end
      end
    end
  end

  return nothing
end

function callback(msg::veh_status,n::NLOpt)
    X0 = zeros(8)
    X0[1] = msg.x_pos
    X0[2] = msg.y_pos
    X0[3] = msg.y_v
    X0[4] = msg.yaw_rate
#    Q = [gs_r.pose.orientation.x,gs_r.pose.orientation.y,gs_r.pose.orientation.z,gs_r.pose.orientation.w]
#    roll, pitch, yaw = tf.euler_from_quaternion(Q)
    X0[5] = msg.yaw_curr
    X0[6] = msg.sa
    X0[7] = msg.x_v
    X0[8] = msg.x_a
    println(string(X0," X0\n"))
    updateX0!(n,X0;(:userUpdate=>true))
end

function main()
  init_node("rosjl_obstacles")
  #linkName = "base_footprint"  # TODO make this a parameter
  modelName = RobotOS.get_param("robotName")

  ###############################
  # set up services and messages

  # pause simulation
  const pause_physics = ServiceProxy("/gazebo/pause_physics",Empty)
  println("Waiting for '/gazebo/pause_physics' service...")
  wait_for_service("/gazebo/pause_physics")

  # unpause simulation
  const unpause_physics = ServiceProxy("/gazebo/unpause_physics",Empty)
  println("Waiting for '/gazebo/unpause_physics' service...")
  wait_for_service("/gazebo/unpause_physics")

  # Set up service to get Gazebo model state
  const get_state = ServiceProxy("/gazebo/get_model_state", GetModelState)
  println("Waiting for 'gazebo/get_model_state' service...")
  wait_for_service("gazebo/get_model_state")

  # Set up service to set Gazebo model state
  const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
  println("Waiting for 'gazebo/set_model_state' service...")
  wait_for_service("gazebo/set_model_state")

  # message for solution to optimal control problem
  const pub = Publisher{Control}("/mavs/optimal_control", queue_size=10)

  # set up services and messages
  ###############################

  c=defineCase(;(:mode=>:autoGazebo));

  # Get the current position of the Gazebo model
  gs = GetModelStateRequest()
  gs.model_name = modelName
  gs_r = get_state(gs)

  if !gs_r.success
      error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
  end

  # Define position to move robot
  vehPose = gs_r.pose  # use the current position
  vehPose.position.x = c.m.X0[1]
  vehPose.position.y = c.m.X0[2]
  roll = 0; pitch = 0; yaw = c.m.X0[5];
  Q = tf.quaternion_from_euler(roll, pitch, yaw)
  vehPose.orientation.x = Q[1]
  vehPose.orientation.y = Q[2]
  vehPose.orientation.z = Q[3]
  vehPose.orientation.w = Q[4]

  #veh_info = Subscriber{veh_status}("vehicleinfo", callback,queue_size = 2)
  # Define the robot state
  ms = ModelState()
  ms.model_name = modelName
  ms.pose = vehPose

  # Set the state of the Gazebo model
  ss = SetModelStateRequest()
  ss.model_state = ms
  println("Calling 'gazebo/set_model_state' service...")
  ss_r = set_state(ss)

  n=initializeAutonomousControl(c);

  driveStraight!(n)

  loop_rate = Rate(2.0)

  # unpause physics
  up = EmptyRequest()
  unpause_physics(up)

idx=1
  while true #!is_shutdown()

    if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.XF_tol[1]
       println("Goal Attained! \n"); n.mpc.goal_reached=true;
       pp = EmptyRequest()
       pause_physics(pp)
       break;
    end

    println("Running model for the: ", n.r.eval_num," time");
    setObstacleData(n.params)
    updateAutoParams!(n,c)  # check to see if goal is in range

    # get the cur6rent position of Gazebo model
    gs = GetModelStateRequest()
    gs.model_name = modelName
    gs_r = get_state(gs)
    if !gs_r.success
        error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
    end
    veh_info = Subscriber{veh_status}("vehicleinfo", callback,(n,),queue_size = 2)

    if idx == 1
        idx=2
        status=autonomousControl!(n)                # rerun optimization
    end
  #  n.mpc.t0_actual=(n.r.eval_num-1)*n.mpc.tex  # external so that it can be updated easily in PathFollowing
    n.mpc.t0_actual = to_sec(get_rostime())
    msg = Control()
    msg.t = n.mpc.t0_actual + n.r.t_st
    msg.x = n.r.X[:,1]
    msg.y = n.r.X[:,2]
    msg.psi = n.r.X[:,5]
    msg.sa = n.r.X[:,6]
    msg.vx = n.r.X[:,7]
    # TODO consider buffering the message here..
    publish(pub,msg)

  #  spinOnce() # call the callbacks
    rossleep(loop_rate) # NOTE might not want to do this ...
  end
end

if !isinteractive()
    main()
end
