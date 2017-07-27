#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Transform
@rosimport tf.msg: tfMessage
rostypegen()
using geometry_msgs.msg
using tf.msg

br = Publisher{tfMessage}("/tf", queue_size=10)

msg=tfMessage()
publish(br,msg)
