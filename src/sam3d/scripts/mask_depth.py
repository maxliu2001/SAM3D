#!/usr/bin/env python
# match depths to generated mask
import rospy
import numpy as np
import re
import open3d as o3d
import cv2
import rospkg

class GeneratePointCLoud:
    def __init__(self):
        self.depth_file = rospkg.RosPack().get_path('sam3d') + "/media/depthmap.npz"
        self.mask_file = rospkg.RosPack().get_path('sam3d') + "/media/masks.npz"
        self.media_file_root = rospkg.RosPack().get_path('sam3d') + "/media/test/"
        self.vis = o3d.visualization.Visualizer()


    def matching(self):
        pcd = o3d.geometry.PointCloud()
        with np.load(self.depth_file) as depth:
            with np.load(self.mask_file) as mask:
                masks = list(mask.keys())
                for each in masks:
                    # open image resource for color channel extraction
                    image = cv2.imread(self.media_file_root+each+'.jpg')
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                    # matching file name to id stored in depth
                    temp = re.compile("([a-zA-Z]+)([0-9]+)")
                    _, res = temp.match(each).groups()
                    
                    # identify index from image where is marked for mask
                    rospy.loginfo(mask[each].shape)
                    indicies = np.where(mask[each])
                    idx = set(zip(indicies[0], indicies[1], indicies[2]))

                    count = 0
                    for _, row, col in idx:
                        # identify color channel for the point
                        color_v = np.array(image[row, col])/255.0
                        # rospy.loginfo(color_v)

                        # generate point cloud coordinates
                        point_v = np.array([[row, col, depth[res][row, col]]])

                        # add pointcloud to visualizer
                        if count == 0:
                            pcd.points = o3d.utility.Vector3dVector(point_v)
                            rospy.loginfo(point_v.shape)
                            pcd.colors = o3d.utility.Vector3dVector([color_v])
                        else:
                            pcd.points.extend(o3d.utility.Vector3dVector(point_v))
                            pcd.colors.extend(o3d.utility.Vector3dVector([color_v]))

                        count += 1
                o3d.io.write_point_cloud(self.media_file_root+"output.ply".format(each), pcd)   
                rospy.loginfo('finished loading matched pcd into ply file.') 

    
    def run(self):
        rospy.init_node('matcher', anonymous=True)
        rospy.loginfo('start matching node')
        self.matching()  

if __name__=='__main__':
    matcher = GeneratePointCLoud()
    matcher.run()
