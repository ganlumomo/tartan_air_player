# ros
import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path

# others
import numpy as np

class TartanAirPlayerNode:
    def __init__(self):
        
        # define camera parameters
        self.fx = 320.0 # focal length x
        self.fy = 320.0 # focal length y
        self.cx = 320.0 # optical center x
        self.cy = 240.0 # optical center y
        self.width = 640
        self.height = 320

        # define msg publishers
        self.pc2_publisher = rospy.Publisher("points", PointCloud2, queue_size = 100)
        self.pose_publisher = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size = 100)

        # run node
        rospy.init_node('tartan_air_player_node', anonymous = True)
        #rospy.Rate(30)
        seq_dir = "/home/ganlu/Downloads/depth_left/seasonsforest/seasonsforest/Easy/P001/"
        left_camera_pose_file = seq_dir + "pose_left.txt"
        self.read_left_camera_poses(left_camera_pose_file)
        depth_left_dir = seq_dir + "depth_left/"
        self.process_scans(depth_left_dir, 100)

    def process_scans(self, depth_left_dir, scan_num):
        for scan_id in range(scan_num):
            rospy.sleep(0.5)
            # load depth img
            depth_left_name = depth_left_dir + "%06i" % scan_id + "_left_depth.npy"
            depth_left = np.load(depth_left_name)
            pc = self.depth_to_pc(depth_left)
            #print(pc)
            #print(pc.shape)
            
            # publish points
            pc2 = self.pc_to_pc2(pc)
            #print(pc2)
            self.pc2_publisher.publish(pc2)

            # publish tf
            print(self.left_camera_poses[scan_id][0])
            br = tf.TransformBroadcaster()
            br.sendTransform((self.left_camera_poses[scan_id][0],
                self.left_camera_poses[scan_id][1],
                self.left_camera_poses[scan_id][2]),
                tf.transformations.quaternion_inverse((self.left_camera_poses[scan_id][3],
                self.left_camera_poses[scan_id][4],
                self.left_camera_poses[scan_id][5],
                self.left_camera_poses[scan_id][6])),
                rospy.Time.now(),
                "map",
                "left_camera")

            # publish poses
            pose = self.pose_with_covariance_stamped(scan_id)
            self.pose_publisher.publish(pose)
            


    def depth_to_pc(self, depth):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        #valid = (depth > 0) & (depth < 255)
        #z = np.where(valid, depth / 256.0, np.nan)
        #x = np.where(valid, z * (c - cx) / fx, 0)
        #y = np.where(valid, z * (r - cy) / fy, 0)
        z = depth
        x = np.where(depth, z * (c - self.cx) / self.fx, 0)
        y = np.where(depth, z * (r - self.cy) / self.fy, 0)
        return np.float32(np.dstack((x, y, z)))

    def pc_to_pc2(self, pc):
        '''Converts a numpy array to a sensor_msgs.msg.PointCloud2'''
        pc2 = PointCloud2()
        pc2.header.frame_id = "left_camera"
        pc2.header.stamp = rospy.Time.now()
        pc2.height = pc.shape[0]
        pc2.width = pc.shape[1]
        pc2.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.is_bigendian = False # assumption
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc.shape[1]
        pc2.is_dense = True
        pc2.data = pc.tostring()
        return pc2

    def read_left_camera_poses(self, left_camera_poses_file):
        self.left_camera_poses = np.loadtxt(left_camera_poses_file)
        #print(self.left_camera_poses)

    def pose(self, scan_id):
        pose = Pose()
        pose.position.x = self.left_camera_poses[scan_id][0]
        pose.position.y = self.left_camera_poses[scan_id][1]
        pose.position.z = self.left_camera_poses[scan_id][2]
        pose.orientation.x = self.left_camera_poses[scan_id][3]
        pose.orientation.y = self.left_camera_poses[scan_id][4]
        pose.orientation.z = self.left_camera_poses[scan_id][5]
        pose.orientation.w = self.left_camera_poses[scan_id][6]
        return pose


    def pose_with_covariance_stamped(self, scan_id):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "left_camera"
        pose.header.stamp = rospy.Time.now()
        # set pose
        pose.pose.pose = self.pose(scan_id)
        # set covariance
        for i in range(36):
            pose.pose.covariance[i] = 0.0
        return pose

    #def add_pose_to_path()
        
    def main(self):
        print("spin..")
        rospy.spin()
        

if __name__ == "__main__":
    node = TartanAirPlayerNode()
    node.main()
