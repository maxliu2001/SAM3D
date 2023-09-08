#!/usr/bin/env python
# match depths to generated mask
import open3d as o3d
import rospy
import rospkg

class VisPointCloud:
    def __init__(self):
        self.media_file_root = rospkg.RosPack().get_path('sam3d') + "/media/test/"
        self.output = self.media_file_root + 'output.ply'
        self.vis = o3d.visualization.Visualizer()
          
    def visualize(self):
        # self.vis.create_window(height=400, width=400)
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #     size=1, origin=[0, 0, 0])
        # self.vis.add_geometry(coordinate_frame)

        pcd = o3d.io.read_point_cloud(self.output)
        o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    vis = VisPointCloud()
    rospy.init_node('visualize', anonymous=True)
    rospy.loginfo('start matching node')
    vis.visualize()