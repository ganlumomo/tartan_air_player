# ros
import rospy

from sensor_msgs.msg import PointCloud2, PointField

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
        self.pc2_publisher = rospy.Publisher("points", PointCloud2, queue_size = 10)

        # run node
        rospy.init_node('tartan_air_player_node', anonymous = True)
        seq_dir = "/home/ganlu/Downloads/depth_left/seasonsforest/seasonsforest/Easy/P001/"
        self.process_scans(seq_dir, 10)

    def process_scans(self, seq_dir, scan_num):
        depth_left_dir = seq_dir + "depth_left/"
        for scan_id in range(scan_num):
            
            # load depth img
            depth_left_name = depth_left_dir + "%06i" % scan_id + "_left_depth.npy"
            depth_left = np.load(depth_left_name)
            pc = self.depth_to_pc(depth_left)
            print(pc.dtype)
            #print(pc)
            #print(pc.shape)
            
            # publish points
            pc2 = self.pc_to_pc2(pc)
            #print(pc2)
            self.pc2_publisher.publish(pc2)

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
        
    def main(self):
        print("spin..")
        rospy.spin()
        

if __name__ == "__main__":
    node = TartanAirPlayerNode()
    node.main()
