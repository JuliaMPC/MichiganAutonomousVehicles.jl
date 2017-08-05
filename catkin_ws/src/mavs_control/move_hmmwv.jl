#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport mavs_control.msg: Control

@rosimport gazebo_msgs.msg: ModelState
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.msg
using gazebo_msgs.srv
using mavs_control.msg

# TODO
# 1) check to see if model is paused
# 2) make a while loop that publishes the current message until there is a new one ready or the current one ran out -> in this case stop the vehicle

function callback(control_msg::Control, get_state::ServiceProxy{GetModelState}, set_state::ServiceProxy{SetModelState})
    modelName = "robot"


    # Get the current position of the Gazebo model
    gs = GetModelStateRequest()
    gs.model_name = modelName
    gs_r = get_state(gs)

    if !gs_r.success
        error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
    end

    # Define position to move robot
    vehPose = gs_r.pose  # use the current position
    vehPose.position.x =
    vehPose.position.y =
    vehPose.orientation.z =

    # Define the robot state
    ms = ModelState()
    ms.model_name = modelName
    ms.pose = vehPose

    # Set the state of the Gazebo model
    ss = SetModelStateRequest()
    ss.model_state = ms
    println("Calling 'gazebo/set_model_state' service...")
    ss_r = set_state(ss)

    if !ss_r.success
        error(string(" calling /gazebo/set_model_state service: ", ss_r.status_message))
    end

end

function main()
    init_node("rosjl_ex")

    # Set up service to set Gazebo model state
    const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
    println("Waiting for 'gazebo/set_model_state' service...")
    wait_for_service("gazebo/set_model_state")

    # Set up service to get Gazebo model state
    const get_state = ServiceProxy("/gazebo/get_model_state", GetModelState)
    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")

    sub_control = Subscriber{Control}("/mavs_control", callback, (set_state, get_state), queue_size = 2)

    spin()
end

if ! isinteractive()
    main()
end
