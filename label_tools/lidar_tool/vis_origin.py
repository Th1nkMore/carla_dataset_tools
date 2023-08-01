import open3d as o3d
import cv2
import numpy as np
import argparse
import math
from pathlib import Path

offset_degree = 0

def config_path():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', type=str, default='1729')
    parser.add_argument('-s', type=str, default='0000404165')
    args = parser.parse_args()


    dir = args.d
    spec = args.s

    pcd = "/home/ghosnp/carla/usable_version_tool/carla_dataset_tools/raw_data/record_2023_"+dir+"/vehicle.tesla.model3_2/velodyne/"+spec+".bin"
    txt = "/home/ghosnp/carla/usable_version_tool/carla_dataset_tools/raw_data/record_2023_"+dir+"/vehicle.tesla.model3_2/velodyne_semantic/"+spec+".txt"

    return pcd,txt


def main():
    pcd_path, txt_path = config_path()
    pre_point = np.fromfile(str(pcd_path), dtype=np.dtype([
                                       ('x', np.float32),
                                       ('y', np.float32),
                                       ('z', np.float32),
                                       ('intensity', np.float32),
                                   ]) ,count=-1)

    pcd = np.array([list(elem) for elem in pre_point])
    
    pred_box = load_pre_label(txt_path) 
    exp_draw_boxes = get_draw_box(pred_box)

    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 800, height = 600)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd[:,:3])
    
    R = np.asarray([[np.cos(offset_degree), -np.sin(offset_degree), 0],
                    [np.sin(offset_degree), np.cos(offset_degree), 0],
                    [0, 0, 1]])
    pcd_o3d.rotate(R, center=(0,0,0))

    # mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1)
    # mesh_sphere.compute_vertex_normals()
    # mesh_sphere.scale(25.0, center=np.array([0, 0, 0]))
    # ball_line_set = o3d.geometry.LineSet.create_from_triangle_mesh(mesh_sphere)
    # ball_line_set.paint_uniform_color([0, 0, 1])
    # vis.add_geometry(ball_line_set)

    vis.add_geometry(mesh)
    vis.add_geometry(pcd_o3d)

    for box in exp_draw_boxes:
        vis.add_geometry(box)

    render_option = vis.get_render_option()
    render_option.point_size = 2
    render_option.background_color = np.asarray([0, 0, 0])



    vis.run()    # vis.add_geometry(line_set)
    vis.destroy_window()



    return


def rotz(t):
    # seems minus 45 degree would have better performance 
    c = np.cos(np.deg2rad(t)-offset_degree)
    s = np.sin(np.deg2rad(t)-offset_degree)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def box2corner(box):
    x = box[0]
    y = box[1]
    z = box[2]
    l = box[3]  # dx
    w = box[4]  # dy
    h = box[5]  # dz
    yaw = box[6]
    Box = np.array(
        [
            [-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
            [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
            [-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2],
        ]
    )

    R = rotz(yaw)
    corners_3d = np.dot(R, Box)  # corners_3d: (3, 8)
    corners_3d[0, :] = corners_3d[0, :] + x
    corners_3d[1, :] = corners_3d[1, :] + y
    corners_3d[2, :] = corners_3d[2, :] + z
    return np.transpose(corners_3d)


def get_line_set(corners):
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],]
    colors = [[1,0,0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def load_pre_label(gt_json_path):
    if Path(gt_json_path).exists():
        pred_box = []
        with open(gt_json_path, 'r') as fobj:
            for line in fobj:

                print("[label_str]",line)

                l = line.strip().split(" ")
                cx, cy, cz, sx, sy, sz, yaw, label = l[0], l[1],l[2], l[3],l[4], l[5],l[6], l[7]
                box_data = list(map(float,[ cx, cy, cz, sx, sy, sz, yaw]))
                pred_box.append(box_data)
        return pred_box

def get_draw_box(pre_box_set):
    draw_boxes = []
    for box_p in pre_box_set:
        cx, cy, cz, sx, sy, sz, yaw = box_p[0],box_p[1],box_p[2], box_p[3],box_p[4], box_p[5],box_p[6]
        corner_box = box2corner([cx, cy, cz, sx, sy, sz, yaw])
        draw_box = get_line_set(corner_box)
        draw_boxes.append(draw_box)
    return draw_boxes




if __name__ == "__main__":
    main()