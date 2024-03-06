import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import tf.transformations

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    name = rospy.get_param('tip', 'base')

    turtle_transform = rospy.Publisher('%s/ele' % name, geometry_msgs.msg.TransformStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(name, 'tip', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # Extract translation and rotation from the original transformation
        translation = [trans.transform.translation.x,
                       trans.transform.translation.y,
                       trans.transform.translation.z]
        rotation = [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w]

        # Convert rotation quaternion to rotation matrix
        R = tf.transformations.quaternion_matrix(rotation)

        # Define the rotation matrix for a 90-degree rotation around the z-axis
        R_z90 = np.array([[0, -1, 0, 0],
                          [1, 0, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Apply additional rotation around z-axis to the rotation matrix
        R = np.dot(R, R_z90)

        # Apply the same rotation to the translation vector (since it's a pure rotation, translation remains the same)
        translation = np.dot(R_z90[:3, :3], translation)

        # Convert the rotation matrix back to a quaternion
        quat = tf.transformations.quaternion_from_matrix(R)

        # Update the rotation and translation components of the transform
        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        trans.transform.translation.x = translation[0]
        trans.transform.translation.y = translation[1]
        trans.transform.translation.z = translation[2]

        # Publish the transformed message
        turtle_transform.publish(trans)

        rate.sleep()
