import open3d as o3d
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
    labels = {}
    # label tag version 0.9.14 (different from early version!!!!!!)
    usable_labels = {7.,12.,14.,15.,16.,19.}

    for point in semantic_point:
        if point[5] in usable_labels:
            if not point[5] in labels:
                labels[point[5]] = {}
            if not point[4] in labels[point[5]]:
                labels[point[5]][point[4]] = []
            labels[point[5]][point[4]].append(point)
            
    corner_set = {}
    for label, points in labels.items():
        for oid, object in points.items():
            x = []
            y = []
            z = []
            for point in object:
                x.append(point[0])
                y.append(point[1])
                z.append(point[2])
            container = [x, y, z]
            np_points = np.array(container).astype(float)
            min_x = np.min(np_points[0,:])
            max_x = np.max(np_points[0,:])
            min_y = np.min(np_points[1,:])
            max_y = np.max(np_points[1,:])
            min_z = np.min(np_points[2,:])
            max_z = np.max(np_points[2,:])
            corners = np.array([[min_x, min_y, min_z],
                                [min_x, min_y, max_z],
                                [min_x, max_y, min_z],
                                [min_x, max_y, max_z],
                                [max_x, min_y, min_z],
                                [max_x, min_y, max_z],
                                [max_x, max_y, min_z],
                                [max_x, max_y, max_z]])
            
            if not label in corner_set:
                corner_set[label] = {}
            corner_set[label][oid] = corners

    return corner_set


def kitti_format_translate(corner_set):

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
                               
    0    Corner Point Format    Points
                                0:[min_x, min_y, min_z],
           7 ------- 5          1:[min_x, min_y, max_z],
         / |       / |          2:[min_x, max_y, min_z],
       3 ------- 1   |          3:[min_x, max_y, max_z],        
       |   |     |   |          4:[max_x, min_y, min_z],        
       |   6 ----|-- 4          5:[max_x, min_y, max_z],        
       | /       | /            6:[max_x, max_y, min_z],        
       2 ------- 0              7:[max_x, max_y, max_z]         

           x    z
            \   |
             \  |
              \ |
       y <----- 0
    """

    label_dict = {7.:'TrafficLight',12.:'Pedestrian',14.:'Car',15.:'Truck',16.:'Bus',19.:'Bicycle'}
    
    label_strs = []
    for label, point_set in corner_set.items():
        for oid, corner in point_set.items():
            k_type = label_dict[label]
            k_truncated = 0
            k_occluded = 0
            k_alpha = 0
            k_bbox = [0,0,0] # skip veloyne project to 2d image
            # dimension (h,w,l)
            k_dimensions = [corner[7][2] - corner[0][2], corner[7][0]-corner[0][0], corner[7][1] - corner[0][1]]
            k_location = [(corner[7][0] + corner[0][0]) / 2, (corner[7][1] + corner[0][1]) / 2, (corner[7][2] + corner[0][2]) / 2]
            k_rotation_y = math.atan(corner[5][1] - corner[1][1]) / (corner[5][0] - corner[1][0])

            # temp function to test format translation

            label_str = "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {}" .format(k_type, k_truncated, k_occluded, k_alpha, 
                                                                                  k_bbox[0], k_bbox[1], k_bbox[2],
                                                                                  k_dimensions[0], k_dimensions[1], k_dimensions[2],
                                                                                  k_location[0], k_location[1], k_location[2],
                                                                                  k_rotation_y, '')
            label_strs.append(label_str)
    return label_strs

def check():
    
    return

def label():
    args = parse_config()
    semantic_point = np.array([list(elem) for elem in np.load(args.data)])

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(semantic_point[:,:3])

    corner_set = get_object_corner(semantic_point)

    lines_box = np.array([[0,1],[1,3],[3,2],[2,0],[5,7],[7,6],[6,4],[5,4],[1,5],[3,7],[0,4],[2,6]])
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

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(semantic_point[:,:3])

    corner_set = get_object_corner(semantic_point)

    lines_box = np.array([[0,1],[1,3],[3,2],[2,0],[5,7],[7,6],[6,4],[5,4],[1,5],[3,7],[0,4],[2,6]])
    colors = np.array([[0, 1, 0] for j in range(len(lines_box))])
    box_set = []

    for label, objects in corner_set.items():
        for oid, corners in objects.items():
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners)
            line_set.lines = o3d.utility.Vector2iVector(lines_box)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            box_set.append(line_set)
            
    return kitti_format_translate(corner_set)

if __name__ == '__main__':
    label()
