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

def get_object_corner(semantic_point,last_dis):
    """                               
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
    total_score = []
    now_dis = {}
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
            if not str(label)+' '+(str(oid)) in last_dis:
                dis = 0
            else: dis = last_dis[str(label)+str(oid)]

            score, dis_k = active_lidar(p_2d,max_z,min_z,dis,label)
            total_score.append(score)
            now_dis[str(label)+str(oid)] = dis_k
            
            # openpcdet format
            label_str = "{} {} {} {} {} {} {} {}" .format(
                                                                rotRect[0][0], rotRect[0][1], (max_z + min_z) / 2,
                                                                rotRect[1][0], rotRect[1][1], (max_z - min_z),
                                                                rotRect[2], label_dict[label])
            
            save_info.append(label_str)

            if not label in corner_set:
                corner_set[label] = {}
            corner_set[label][oid] = corners

    # print(sum(total_score) / len(total_score))
    return save_info, corner_set, now_dis, sum(total_score) / len(total_score)

def main():
    args = parse_config()
    semantic_point = np.array([list(elem) for elem in np.load(args.data)])

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(semantic_point[:,:3])

    _, corner_set, _, _ = get_object_corner(semantic_point,{})

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

def save_label(lidar_data, last_score):
    semantic_point = np.array([list(elem) for elem in lidar_data])

    label_set, _, now_dis, score = get_object_corner(semantic_point,last_score)
    return label_set, now_dis, score

def active_lidar(p_2d,max_z,min_z,dis, type):
    # TODO: fix active frequency
    rotRect = cv2.minAreaRect(np.array(p_2d,dtype=np.float32))
    dis_k = math.sqrt(math.pow(rotRect[0][0], 2) + math.pow(rotRect[0][1], 2)+math.pow((max_z + min_z) / 2 , 2))
    
    s_dis = score_dis(dis_k)
    s_rho = score_rho(rotRect,len(p_2d),max_z,min_z)
    if type in {14.,15.,16.,19.} and dis != 0:
        s_tau = score_tau(dis_k - dis)
    else: s_tau = 0

    score = score_total(s_dis, s_rho, s_tau)
    return score,dis_k

def score_dis(x):
    s = 1 / 90 * max(0, 0.0001 * math.pow(x,3) - 0.0578 * math.pow(x, 2) + 4.8141 * x - 19.495)
    return s

def score_rho(infos,count,max_z,min_z):
    div = infos[1][0] * infos[1][1] * (max_z - min_z)
    if div == 0: return 0
    s = count /div
    if s < 35 and s > 20: return 1
    return 0.65 

def score_tau(x):
    # sigmonid function
    return 1 / (1 + math.pow(math.e, -x))

def score_total(s_dis, s_rho, s_tau):
    return 0.8 * s_dis + 0.8 * s_rho + 1.4 * s_tau

if __name__ == '__main__':
    # use -d to input .npy velodyne data path
    main()
