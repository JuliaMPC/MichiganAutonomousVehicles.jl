#!/usr/bin/env julia
using RobotOS
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle
rostypegen()
using obstacle_detector.msg

function cb(msg::Obstacles)
L = length(msg.circles)
  for i in 1:L
    println("the ", i, "th : ", msg.circles[i].center.x)
  end
end

function main()
  init_node("rosjl_obstacles")
  sub = Subscriber{Obstacles}("/obstacles", cb, queue_size = 10)
end

main()

spin()
