#!/usr/bin/python3

import open3d as o3d
import cv2
import carla
import time
import numpy as np
import label_tools.kitti_lidar.lidar_label_view as label_tool
import label_tools.lidar_tool.util as util
from recorder.sensor import Sensor


class Lidar(Sensor):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        # Save as a Nx4 numpy array. Each row is a point (x, y, z, intensity)
        lidar_data = np.fromstring(bytes(sensor_data.raw_data),
                                   dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))

        # Convert point cloud to right-hand coordinate system
        lidar_data[:, 1] *= -1

        # Save point cloud to [RAW_DATA_PATH]/.../[ID]_[SENSOR_TYPE]/[FRAME_ID].npy
        # np.save("{}/{:0>10d}".format(save_dir, sensor_data.frame), lidar_data)
        with open("{}/{:0>10d}.bin".format(save_dir,sensor_data.frame), 'wb') as file:
            file.write(lidar_data)
        return True


class SemanticLidar(Sensor):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)
        self.dis_dict = {}

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        # TODO: make tools here

        # Save data as a Nx6 numpy array.
        lidar_data = np.fromstring(bytes(sensor_data.raw_data),
                                   dtype=np.dtype([
                                       ('x', np.float32),
                                       ('y', np.float32),
                                       ('z', np.float32),
                                       ('CosAngle', np.float32),
                                       ('ObjIdx', np.uint32),
                                       ('ObjTag', np.uint32)
                                   ]))

        # Convert point cloud to right-hand coordinate system
        lidar_data['y'] *= -1
        tick_s = time.time()
        # Save point cloud to [RAW_DATA_PATH]/.../[ID]_[SENSOR_TYPE]/[FRAME_ID].npy
        # dataset, now_dis, score = label_tool.save_label(lidar_data, self.dis_dict)
        # self.dis_dict = now_dis

        labels = self.get_label()




        if True:
            # np.save("{}/{:0>10d}".format(save_dir,sensor_data.frame),lidar_data)
            with open("{}/{:0>10d}.bin".format(save_dir,sensor_data.frame), 'wb') as file:
                file.write(lidar_data)
            with open("{}/{:0>10d}.txt".format(save_dir,sensor_data.frame),'a+') as f:
                for line in labels:
                    print(line,file=f)
        return True
    

    def get_label(self):
        labels = []
        bbox_dict,trans_dict,tags_dict,trans_vehicle = self.get_near_bounding_box()
        bbox_vehicle = self.parent.get_carla_bbox()
        for key in bbox_dict:
            temp_bbox = bbox_dict[key]
            temp_trans = trans_dict[key]
            temp_tag = tags_dict[key]

            # update: use sensor actor position

            delta_pose = np.array([temp_trans.location.x - trans_vehicle.location.x,temp_trans.location.y - trans_vehicle.location.y , temp_trans.location.z - trans_vehicle.location.z ])
            

            cx = -(delta_pose[1]+temp_bbox.location.y)
            cy = -(delta_pose[0]+temp_bbox.location.x)
            cz = delta_pose[2]+temp_bbox.location.z
            sx = 2*temp_bbox.extent.x
            sy = 2*temp_bbox.extent.y
            sz = 2*temp_bbox.extent.z
            rotation_y = -(temp_trans.rotation.yaw - trans_vehicle.rotation.yaw + temp_bbox.rotation.yaw)

            label_str = "{} {} {} {} {} {} {} {}" .format(cx, cy, cz, sx, sy, sz, rotation_y, "Car-" + str(key) + str(temp_tag))

            # label_str = "{} {} {} {} {} {} {} {}" .format(delta_pose[0] + temp_bbox.location.x, delta_pose[1]+temp_bbox.location.y, delta_pose[2]+temp_bbox.location.z,
            #                                                     2*temp_bbox.extent.x, 2*temp_bbox.extent.y, 2*temp_bbox.extent.z,
            #                                                     temp_trans.rotation.yaw - trans_vehicle.rotation.yaw + temp_bbox.rotation.yaw, "Car-" + str(key))
            labels.append(label_str)
        
        return labels
    
    def set_car_list(self, record_cars, other_cars):
        self.record_cars = record_cars
        self.other_cars = other_cars

    def get_near_bounding_box(self):
        bbox_dict = {}
        trans_dict={}
        tags_dict = {}
        # get the bounding box of the near car
        for vehicle in self.record_cars:
            print("="*80)
            print("[vehicle]",vehicle.get_carla_transform())
            print("[sensor]",self.carla_actor.get_transform())
            print("="*80)
            for npc in self.other_cars:       
                # sem_tag = npc.get_carla_actor().semantic_tags
                # for tag in sem_tag:
                #     if tag in {"17","18","19","21","22"}:
                #         print("[npc]",npc.get_carla_actor().id,npc.get_carla_actor().semantic_tags)
                #         break
                dist = npc.get_carla_transform().location.distance(vehicle.get_carla_transform().location)
                if dist < 30:
                    bbox_dict[npc.get_carla_actor().id]  = npc.get_carla_bbox()
                    trans_dict[npc.get_carla_actor().id] = npc.get_carla_transform()
                    tags_dict[npc.get_carla_actor().id] = npc.get_carla_actor().semantic_tags

        return bbox_dict,trans_dict, tags_dict,self.carla_actor.get_transform()


    def project_3D_to_2D(self,img,world_2_camera):
        # project 3D bounding box in 2D image, need carla camera inner infos(node) and output image
        # TODO: fix the bug, output as a tuple or string in kitti format

        # edge pairs used to visualize
        edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

        K = self.build_projection_matrix(800, 600, 90)
        verts_2D = self.get_near_bounding_box()
        for verts in verts_2D:
            for edge in edges:
                p1 = self.get_image_point(verts[edge[0]], K, world_2_camera)
                p2 = self.get_image_point(verts[edge[1]],  K, world_2_camera)
                # TODO: using open3d draw lines and visualize
                cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        
                

    def build_projection_matrix(w, h, fov):
        # default: w = 800，h = 600，fov = 90
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        K = np.identity(3)
        K[0, 0] = K[1, 1] = focal
        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        return K
    
    def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]