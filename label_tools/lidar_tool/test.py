# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
from pathlib import Path

def rotz(t):
    c = np.cos(t)
    s = np.sin(t)
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
    # 先旋转再平移
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
            print("load pre label: ", gt_json_path)
            for line in fobj:
                l = line.strip().split(" ")
                cx, cy, cz, sx, sy, sz, yaw = l[0], l[1],l[2], l[3],l[4], l[5],l[6]
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


if __name__ == '__main__':
    exp_gt_pcd = r"~/test/833.9.pcd"
    exp_pred_json = r"~/test/833predected.txt"
    mesh_frame = o3d.io.read_point_cloud(exp_gt_pcd, remove_infinite_points=True, remove_nan_points=True)
    box_sigle_pcd_all = load_pre_label(exp_pred_json) 
    exp_draw_boxes = get_draw_box(box_sigle_pcd_all)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="show_pred_pcd")
    render_option = vis.get_render_option()
    render_option.point_size = 2
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
    vis.add_geometry(coordinate_frame)
    vis.add_geometry(mesh_frame)  
    for box in exp_draw_boxes:
        vis.add_geometry(box)
    vis.run()
    vis.destroy_window()