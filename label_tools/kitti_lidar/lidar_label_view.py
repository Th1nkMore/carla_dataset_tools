import open3d as o3d
import cv2
import numpy as np
import argparse
import math

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--data','-d',type=str,help='specify the point cloud data file or directory')
    args = parser.parse_args()
  
    return args

def custom_draw_geometry(pcd,box_set):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    for line_set in box_set:
        vis.add_geometry(line_set)
    render_option = vis.get_render_option()
    render_option.point_size = 4
    render_option.background_color = np.asarray([0, 0, 0])
    vis.run()
    vis.destroy_window()

def get_object_corner(semantic_point):
    """
    #Values    Name      Description
    ----------------------------------------------------------------------------
    1    type         Describes the type of object: 'Car', 'Pedestrian', 'Vehicles'
                        'Vegetation', 'TrafficSigns', etc.
    1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                        truncated refers to the object leaving image boundaries
    1    occluded     Integer (0,1,2,3) indicating occlusion state:
                        0 = fully visible, 1 = partly occluded
                        2 = largely occluded, 3 = unknown
    1    alpha        Observation angle of object, ranging [-pi..pi]
    4    bbox         2D bounding box of object in the image (0-based index):
                        contains left, top, right, bottom pixel coordinates
    3    dimensions   3D object dimensions: height, width, length (in meters)
    3    location     3D object location x,y,z in camera coordinates (in meters)
    1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
    1    score        Only for results: Float, indicating confidence in
                        detection, needed for p/r curves, higher is better.
                               
    0    Corner Point Format
                                
           7 ------- 5          
         / |       / |          
       1 ------- 3   |                  
       |   |     |   |          
       |   6 ----|-- 4                
       | /       | /              
       0 ------- 2                 

           x    z
            \   |
             \  |
              \ |
       y <----- 0
    """
    labels = {}
    # label tag version 0.9.14 (different from early version!!!!!!)
    usable_labels = {7.,12.,14.,15.,16.,19.}
    label_dict = {7.:'TrafficLight',12.:'Pedestrian',14.:'Car',15.:'Truck',16.:'Bus',19.:'Bicycle'}

    for point in semantic_point:
        if point[5] in usable_labels:
            if not point[5] in labels:
                labels[point[5]] = {}
            if not point[4] in labels[point[5]]:
                labels[point[5]][point[4]] = []
            labels[point[5]][point[4]].append(point)
            
    save_info = []
    corner_set={}
    for label, points in labels.items():
        for oid, object in points.items():
            z = []
            for point in object:
                z.append(point[2])
            min_z = np.min(z)
            max_z = np.max(z)
            p_2d = []
            for p in object:
                p_2d.append([p[0],p[1]])
            rotRect = cv2.minAreaRect(np.array(p_2d,dtype=np.float32))
            corner_point = cv2.boxPoints(rotRect)
            corners = np.array([[corner_point[0][0], corner_point[0][1], min_z],
                              [corner_point[0][0], corner_point[0][1], max_z],
                              [corner_point[1][0], corner_point[1][1], min_z],
                              [corner_point[1][0], corner_point[1][1], max_z],
                              [corner_point[2][0], corner_point[2][1], min_z],
                              [corner_point[2][0], corner_point[2][1], max_z],
                              [corner_point[3][0], corner_point[3][1], min_z],
                              [corner_point[3][0], corner_point[3][1], max_z]])
            # kitti format
            # label_str = "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {}" .format(label_dict[label], 0, 0, 0, 0, 0, 0,
            #                                                                 (max_z - min_z), rotRect[1][0], rotRect[1][1],
            #                                                                 rotRect[0][0], rotRect[0][1], (max_z + min_z) / 2,
            #                                                                 rotRect[2], '')
            
            # openpcdet format
            label_str = "{} {} {} {} {} {} {} {}" .format(
                                                                rotRect[0][0], rotRect[0][1], (max_z + min_z) / 2,
                                                                rotRect[1][0], rotRect[1][1], (max_z - min_z),
                                                                rotRect[2], label_dict[label])
            
            save_info.append(label_str)

            if not label in corner_set:
                corner_set[label] = {}
            corner_set[label][oid] = corners

    return save_info, corner_set

def check():
    
    return

def label():
    args = parse_config()
    semantic_point = np.array([list(elem) for elem in np.load(args.data)])

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(semantic_point[:,:3])

    _, corner_set = get_object_corner(semantic_point)

    # for old version
    # lines_box = np.array([[0,1],[1,3],[3,2],[2,0],[5,7],[7,6],[6,4],[5,4],[1,5],[3,7],[0,4],[2,6]])

    # for new version
    lines_box = np.array([[0,1],[2,3],[4,5],[6,7],[0,2],[6,4],[1,3],[7,5],[0,6],[2,4],[1,7],[3,5]])
    
    
    colors = np.array([[0, 1, 0] for j in range(len(lines_box))])
    box_set = []

    for label, objects in corner_set.items():
        for oid, corners in objects.items():
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners)
            line_set.lines = o3d.utility.Vector2iVector(lines_box)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            box_set.append(line_set)

    custom_draw_geometry(point_cloud, box_set)

def save_label(lidar_data):
    semantic_point = np.array([list(elem) for elem in lidar_data])

    label_set, _ = get_object_corner(semantic_point)
    return label_set

if __name__ == '__main__':
    label()
