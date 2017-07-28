#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Transform, TransformStamped, Quaternion, Vector3
@rosimport tf.msg: tfMessage
@rosimport tf2_msgs.msg: TFMessage
rostypegen()
using geometry_msgs.msg
using tf.msg
using tf2_msgs.msg


init_node("tf_jl")

br = Publisher{TFMessage}("/tf", queue_size = 10)

# time
#ros_time = get_rostime()

tfs = TransformStamped()
#tfs.header.stamp.secs = to_sec(ros_time)
#tfs.header.stamp.nsecs = to_nsecs(ros_time)
tfs.header.frame_id = "base_footprint"
tfs.child_frame_id = "velodyne_top_link"
tfs.transform.translation = Vector3(rand(), 0.0, 0.0) # TODO get the current position
tfs.transform.rotation = Quaternion(0.0, rand(), 0.0, 0.0)
tf_msg = TFMessage()
push!(tf_msg.transforms,tfs)
publish(br,tf_msg)

cb(tf_msg::TFMessage) = println(string("run:", tf_msg.msg.tfMessage))
cj = Subscriber{TFMessage}("/tf", cb, queue_size = 10)
spin()
