import open3d as o3d
import os
import sys
import numpy as np
import argparse
import math

usable_labels = {12.,14.,15.,16.,19.}
label_dict = {12.:'Pedestrian',14.:'Car',15.:'Truck',16.:'Bus',19.:'Bicycle'}

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--data','-d',type=str,help='specify the point cloud data file or directory')
    parser.add_argument('--type','-t',type=str,default='bin',help='specify the point cloud data file or directory')
    
    args = parser.parse_args()
  
    return args

def split_object(semantic_points):
    ground = []
    nonground = []
    objects = []
    objects_dict = {}

    for point in semantic_points:
        if point[5] in usable_labels:
            objects.append([point[0],point[1],point[2]])
            if not point[5] in objects_dict:
                objects_dict[point[5]] = {}
            if not point[4] in objects_dict[point[5]]:
                objects_dict[point[5]][point[4]] = []
            objects_dict[point[5]][point[4]].append(point)
        elif point[5] == 1.:
            ground.append([point[0],point[1],point[2]])
        else:
            nonground.append([point[0],point[1],point[2]])

    return ground,nonground,objects,objects_dict

def get_object_corner(objects_dict,last_dist_dict,fitter):
    save_info = []
    corner_set={}
    dist_dict = {}
    for label, points in objects_dict.items():
        for oid, object_points in points.items():
            max_p = np.max(object_points, axis=0)
            min_p = np.min(object_points, axis=0)
            max_x = max_p[0]
            min_x = min_p[0]
            max_y = max_p[1]
            min_y = min_p[1]
            max_z = max_p[2]
            min_z = min_p[2]
            
            pre_distance = math.sqrt(((max_x + min_x)/2)**2 + ((max_y + min_y)/2)**2 + ((max_z + min_z)/2)**2)

            reason = " pass  "
            if pre_distance < 50:
                p_2d = np.array(object_points)[:,:2]
                cnt = len(p_2d)
                size = (max_x - min_x)*(max_y - min_y)
                if size > 1 and cnt / size > 45:
                    if_collect = True
                elif size > 0.7 and cnt/size > 10:
                    if not str(label)+" "+str(oid) in last_dist_dict:
                        if_collect = True
                        reason = "appear"
                    else:
                        last_distance = last_dist_dict[str(label)+" "+str(oid)]
                        var = 10 * math.min(pre_distance,last_distance) / math.max(pre_distance,last_distance)
                        sigmoid_res = abs(1 / (1 + math.pow(math.e,-var)))
                        if_collect = sigmoid_res > 0.8
                        if sigmoid_res > 0.8:
                            if_collect = True
                            reason = " pass  "
                        else:
                            if_collect = False
                            reason="move  "
                else:
                    if_collect = False
                    reason = "rho   "

                if if_collect:
                    corner_point = fitter.get_rectangle(p_2d,label).calc_rect_contour()
                    corners = np.array([[corner_point[0][0], corner_point[1][0], min_z],
                                    [corner_point[0][0], corner_point[1][0], max_z],
                                    [corner_point[0][1], corner_point[1][1], min_z],
                                    [corner_point[0][1], corner_point[1][1], max_z],
                                    [corner_point[0][2], corner_point[1][2], min_z],
                                    [corner_point[0][2], corner_point[1][2], max_z],
                                    [corner_point[0][3], corner_point[1][3], min_z],
                                    [corner_point[0][3], corner_point[1][3], max_z]])
                    center = [(corners[4][0]+corners[0][0])/2,(corners[4][1]+corners[0][1])/2,(corners[4][2]+corners[0][2])/2]
                    distance = math.sqrt(center[0]**2 + center[1]**2 + center[2]**2)
                    dist_dict[str(label)+" "+str(oid)] = distance
                    l = np.sqrt((corners[2][0] - corners[0][0])**2 + (corners[2][1] - corners[0][1])**2)
                    w = np.sqrt((corners[6][0] - corners[0][0])**2 + (corners[6][1] - corners[0][1])**2)
                    h = max_z-min_z
                    rotation_y = np.arctan((corners[2][1] - corners[0][1]) / (corners[2][0] - corners[0][0]))

                    label_str = "{} {} {} {} {} {} {} {} {}" .format(center[0], center[1], center[2],
                                                                        l, w, h,
                                                                        rotation_y, label_dict[label], oid)

                    save_info.append(label_str)
                    print("[label_str]",label_str)

                    if not label in corner_set:
                        corner_set[label] = {}
                    corner_set[label][oid] = corners
                # print(if_collect,reason, pre_distance, size  ,cnt / size, oid)

    return save_info, corner_set, dist_dict

def save_label(lidar_data, last_dist_dict,fitter):
    semantic_point = np.array([list(elem) for elem in lidar_data])
    _, _, _, objects_dict = split_object(semantic_point)
    label_set, _, dist_dict = get_object_corner(objects_dict,last_dist_dict, fitter)
    return label_set, dist_dict, 1

# ******************************************************************************
# [Visualization Point Cloud Here]
# ******************************************************************************

def main():
    import util
    args = parse_config()
    if args.type == 'npy':
        pre_point = np.load(args.data)
    elif args.type == 'bin':
        pre_point = np.fromfile(str(args.data), dtype=np.dtype([
                                       ('x', np.float32),
                                       ('y', np.float32),
                                       ('z', np.float32),
                                       ('CosAngle', np.float32),
                                        ('ObjIdx', np.uint32),
                                       ('ObjTag', np.uint32)
                                   ]) ,count=-1)
        # print(pre_point)

    semantic_point = np.array([list(elem) for elem in pre_point])
    ground,nonground,dect_objects,objects_dict = split_object(semantic_point)
    _, corner_set, _= get_object_corner(objects_dict,{},util.LShapeFitting())

    open3d_draw_picture(ground,nonground,dect_objects,corner_set)

def open3d_draw_picture(ground,nonground,dect_objects,corner_set):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 800, height = 600)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    ground_o3d = o3d.geometry.PointCloud()
    ground_o3d.points = o3d.utility.Vector3dVector(ground)
    ground_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[0.0, 0.8, 0.2] for _ in range(len(ground))])
    )

    nonground_o3d = o3d.geometry.PointCloud()
    nonground_o3d.points = o3d.utility.Vector3dVector(nonground)

    object_o3d = o3d.geometry.PointCloud()
    object_o3d.points = o3d.utility.Vector3dVector(dect_objects)
    object_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[1.0, 0.0, 0.0] for _ in range(len(dect_objects))])
    )

    lines_box = np.array([[0,1],[2,3],[4,5],[6,7],[0,2],[6,4],[1,3],[7,5],[0,6],[2,4],[1,7],[3,5]])
    colors = np.array([[0, 1, 1] for j in range(len(lines_box))])
    box_set = []
    for label, objects in corner_set.items():
        for oid, corners in objects.items():
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners)
            line_set.lines = o3d.utility.Vector2iVector(lines_box)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            box_set.append(line_set)
    for line_set in box_set:
        vis.add_geometry(line_set)

    # mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1)
    # mesh_sphere.compute_vertex_normals()
    # mesh_sphere.scale(25.0, center=np.array([0, 0, 0]))
    # ball_line_set = o3d.geometry.LineSet.create_from_triangle_mesh(mesh_sphere)
    # ball_line_set.paint_uniform_color([0, 0, 1])
    # vis.add_geometry(ball_line_set)

    vis.add_geometry(mesh)
    vis.add_geometry(ground_o3d)
    vis.add_geometry(nonground_o3d)
    vis.add_geometry(object_o3d)

    render_option = vis.get_render_option()
    render_option.point_size = 2
    render_option.background_color = np.asarray([0, 0, 0])

    vis.run()
    vis.destroy_window()

# ******************************************************************************
# [Active strategy Here]
# ******************************************************************************

def active_voxel_entroy(origin_entropy, pcd):
    import util
    voxel = util.Voxel()
    now_entropy = voxel.get_entropy_score(pcd)
    return abs((now_entropy - origin_entropy) / origin_entropy) > 0.2, now_entropy


if __name__ == '__main__':
    main()
