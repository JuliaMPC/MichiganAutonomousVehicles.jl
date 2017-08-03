#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg

function callback(twist_msg::Twist, get_state::ServiceProxy{GetModelState}, set_state::ServiceProxy{SetModelState})
    modelName = "robot"  # TODO make this a parameter

    # Get the current position of the Gazebo model
    gs = GetModelStateRequest()
    gs.model_name = modelName
    gs_r = get_state(gs)

    if !gs_r.success
        error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
    end

    # Define the robot state
    ms = ModelState()
    ms.model_name = modelName
    ms.pose = gs_r.pose
    ms.twist = twist_msg

    # Set the state of the Gazebo model
    ss = SetModelStateRequest()
    ss.model_state = ms
    ss_r = set_state(ss)

    if !ss_r.success
        error(string(" calling /gazebo/set_model_state service: ", ss_r.status_message))
    end

end

function main()
    init_node("rosjl_example")

    # Set up service to set Gazebo model state
    const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
    println("Waiting for 'gazebo/set_model_state' service...")
    wait_for_service("gazebo/set_model_state")

    # Set up service to get Gazebo model state
    const get_state = ServiceProxy("/gazebo/get_model_state", GetModelState)
    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")

    sub = Subscriber{Twist}("/cmd_vel",callback,(get_state,set_state), queue_size=10)  # pub is the additional callback param is given to forward on
    spin()
end

if ! isinteractive()
    main()
end


#= TODO

# Teliop
function callback(msg::Twist, pub_obj::Publisher{ModelState)
    pt_msg = Pose(msg.linear.x, msg.linear.x, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)

    # Define the robot state
    ms = ModelState()
    ms.model_name = modelName
    ms.pose = vehPose

    publish(pub_obj, pt_msg)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Point(rand(), rand(), 0.0)
        publish(pub_obj, npt)
        rossleep(loop_rate)
    end
end
function main()
    init_node("rosjl_example")
    pub = Publisher{"/gazebo/model_states", ModelState}
    sub = Subscriber{Twist}("/cmd_vel",callback,(pub,), queue_size=10)  # pub is the additional callback param is given to forward on
    loop(pub)
end

if ! isinteractive()
    main()
end

# Teliop
# wait for all services to be in some ready state, seehttp://docs.ros.org/electric/api/srs_user_tests/html/prepare__robot__manip__sim_8py_source.html
# start laucnh file in a paused state and have some sin_init file start it

 sim = get_param("/use_sim_time")
 if sim
      loginfo('Waiting until simulated robot is prepared for the task...')


      const get_world = ServiceProxy("/gazebo/get_world_properties",GetWorldProperties)
      println("Waiting for '/gazebo/get_world_properties' service...")
      wait_for_service("/gazebo/get_world_properties")

      wp_r=GetWorldPropertiesRequest()

      wp=get_world(wp_r)

wait_for_message('/sim_robot_init',EmptyMsg)
wait_for_service("/gazebo/pause_physics")

loginfo("Pausing physics")

=#
# world files with obstacles
# LiDAR data processing algorithm
# figure out elipses
