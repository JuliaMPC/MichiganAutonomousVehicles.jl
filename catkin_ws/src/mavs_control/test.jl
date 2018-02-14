#!/usr/bin/env julia
using RobotOS
@rosimport mavs_control.msg: Control
rostypegen()
using mavs_control.msg

#http://wiki.ros.org/message_filters#Time_Synchronizer


# Set up service to get Gazebo link state
const get_link_state = ServiceProxy("/gazebo/get_link_state",GetLinkState)
println("Waiting for 'gazebo/get_link_state' service...")
wait_for_service("gazebo/get_link_state")


function main()
  init_node("ros_ex")
  const pub = Publisher{Control}("/mavs/optimal_control", queue_size=10)
  loop_rate = Rate(5.0)
  while !is_shutdown()
    msg = Control()
    msg.sa = [1.0, 2.0]
    msg.vx = [ 1.9, 3.0]
    msg.x = [ 1.9, 3.0]
    msg.y = [ 1.9, 3.0]
    msg.psi = [ 1.9, 3.0]
    publish(pub,msg)
    rossleep(loop_rate)
  end
end

main()
