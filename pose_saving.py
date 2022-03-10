
import rospy
from tf.transformations import quaternion_from_euler


y_ls = ["0.0", "0,3", "-0.3"]
rot_ls = ["0.0", "7.0", "-7.0"]

for i, y_wp in enumerate(y_ls):
        z_rot = (rot_ls[i])
        q_rot = quaternion_from_euler(0, 0, z_rot)
        print(i)
        print(q_rot,y_wp)

print ("done")
