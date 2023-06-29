#!/usr/bin/python3
import csv
import carla
import cv2 as cv
import numpy as np
import transforms3d
import math

from recorder.sensor import Sensor
from utils.geometry_types import Transform, Rotation
from utils.transform import carla_transform_to_transform


class CameraBase(Sensor):
    def __init__(self,
                 uid,
                 name: str,
                 base_save_dir: str,
                 parent,
                 carla_actor: carla.Sensor,
                 color_converter: carla.ColorConverter = None):
        super().__init__(uid, name, base_save_dir, parent, carla_actor)
        self.edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]
        self.color_converter = color_converter

    def save_to_disk_impl(self, save_dir, sensor_data) -> bool:
        # Convert to target color template
        if self.color_converter is not None:
            sensor_data.convert(self.color_converter)
        img = np.reshape(np.copy(sensor_data.raw_data), (sensor_data.height, sensor_data.width, 4))
        with open("{}/{:0>10d}.txt".format(save_dir, sensor_data.frame),'a+') as f:
            pass
        # Get the camera matrix 
        try:
            count = 0
            world_2_camera = np.array(self.carla_actor.get_transform().get_inverse_matrix())
            vehicle = self.parent
            world = vehicle.carla_world
            image_w = int(self.carla_actor.attributes['image_size_x'])
            image_h = int(self.carla_actor.attributes['image_size_y'])
            fx = image_w / (
                2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
            K = self.build_projection_matrix(image_w, image_h, fx)
            count = 0
            for npc in world.get_actors().filter('*vehicle*'):
                if npc.id != vehicle.uid:
                    bb = npc.bounding_box
                    dist = npc.get_transform().location.distance(vehicle.get_carla_transform().location)
                    # Filter for the vehicles within 50m
                    if dist < 50:
                    # Calculate the dot product between the forward vector
                    # of the vehicle and the vector between the vehicle
                    # and the other vehicle. We threshold this dot product
                    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                        forward_vec = vehicle.get_carla_transform().get_forward_vector()
                        ray = npc.get_transform().location - vehicle.get_carla_transform().location
                        if forward_vec.dot(ray) > 1:
                            verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                            x_max = -10000
                            x_min = 10000
                            y_max = -10000
                            y_min = 10000

                            for vert in verts:
                                p = self.get_image_point(vert, K, world_2_camera)
                                # Find the rightmost vertex
                                if p[0] > x_max:
                                    x_max = p[0]
                                # Find the leftmost vertex
                                if p[0] < x_min:
                                    x_min = p[0]
                                # Find the highest vertex
                                if p[1] > y_max:
                                    y_max = p[1]
                                # Find the lowest  vertex
                                if p[1] < y_min:
                                    y_min = p[1]
                            center_x = ((int(x_max) + int(x_min))/2)/int(sensor_data.width)
                            center_y = ((int(y_max) + int(y_min))/2)/int(sensor_data.height)
                            bbox_x = (int(x_max) - int(x_min))/int(sensor_data.width)
                            bbox_y = (int(y_max) - int(y_min))/int(sensor_data.height)
                            if(1>center_x>0 and 1>center_y>0 and 1>bbox_x>0 and 1>bbox_y>0):
                                count += self.complexity(npc, dist)
                                # cv.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                                # cv.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                                # cv.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                                # cv.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                                count+=1
                                with open("{}/{:0>10d}.txt".format(save_dir,
                                                        sensor_data.frame),'a+') as f:
                                    f.write(f"0 {center_x} {center_y} {bbox_x} {bbox_y}\n")
            # cv.putText(img, f"complexity: {count} ", (10, 50), cv.FONT_HERSHEY_TRIPLEX, 1, (0,0,255, 255), 2)
            # cv.putText(img, str(world.get_weather()), (10, 100), cv.FONT_HERSHEY_TRIPLEX, 1, (0,0,255, 255), 2)

        except Exception as e:
            print("test:"+e)

        # Convert raw data to numpy array, image type is 'bgra8'
        # carla_image_data_array = np.ndarray(shape=(sensor_data.height,
        #                                            sensor_data.width,
        #                                            4),
        #                                     dtype=np.uint8,
        #                                     buffer=sensor_data.raw_data)

        # Save image to [RAW_DATA_PATH]/.../[ID]_[SENSOR_TYPE]/[FRAME_ID].png
        if count >=1:
            success = cv.imwrite("{}/{:0>10d}.png".format(save_dir,
                                                        sensor_data.frame),
                                img)

        if self.is_first_frame():
            self.save_camera_info(save_dir)

        return success

    def save_camera_info(self, save_dir):
        with open('{}/camera_info.csv'.format(save_dir), 'w', encoding='utf-8') as csv_file:
            fieldnames = {'width',
                          'height',
                          'fx',
                          'fy',
                          'cx',
                          'cy'}
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            camera_info = self.get_camera_info()
            writer.writerow(camera_info)

    def get_camera_info(self):
        camera_width = int(self.carla_actor.attributes['image_size_x'])
        camera_height = int(self.carla_actor.attributes['image_size_y'])
        fx = camera_width / (
                2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
        return {
            'width': camera_width,
            'height': camera_height,
            'cx': camera_width / 2.0,
            'cy': camera_height / 2.0,
            'fx': fx,
            'fy': fx
        }

    def get_transform(self) -> Transform:
        c_trans = self.carla_actor.get_transform()
        trans = carla_transform_to_transform(c_trans)
        quat = trans.rotation.get_quaternion()
        quat_swap = transforms3d.quaternions.mat2quat(np.matrix(
                      [[0, 0, 1],
                       [-1, 0, 0],
                       [0, -1, 0]]))
        quat_camera = transforms3d.quaternions.qmult(quat, quat_swap)
        roll, pitch, yaw = transforms3d.euler.quat2euler(quat_camera)
        return Transform(trans.location, Rotation(roll=math.degrees(roll),
                                                  pitch=math.degrees(pitch),
                                                  yaw=math.degrees(yaw)))

    def build_projection_matrix(self, w, h, fx):
        K = np.identity(3)
        K[0, 0] = K[1, 1] = fx
        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        return K

    # Calculate 2D projection of 3D coordinate
    def get_image_point(self, loc, K, w2c):

        point = np.array([loc.x, loc.y, loc.z, 1])
        point_camera = np.dot(w2c, point)

        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        point_img = np.dot(K, point_camera)

        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

    def complexity(self, npc, dist):
        t = npc.get_velocity()
        velocity = 3.6 * math.sqrt(t.x * t.x                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
                               + t.y * t.y
                               + t.z * t.z)
        return velocity + (0 if dist == 0 else min(50, 1/dist**2))

class RgbCamera(CameraBase):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor,
                 color_converter: carla.ColorConverter = None):
        super().__init__(uid, name, base_save_dir, parent, carla_actor, color_converter)

class SemanticSegmentationCamera(CameraBase):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor,
                 color_converter: carla.ColorConverter = None):
        color_converter = carla.ColorConverter.CityScapesPalette
        super().__init__(uid, name, base_save_dir, parent, carla_actor, color_converter)

class DepthCamera(CameraBase):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor,
                 color_converter: carla.ColorConverter = None):
        color_converter = carla.ColorConverter.Depth
        super().__init__(uid, name, base_save_dir, parent, carla_actor, color_converter)

class InstanceCamera(CameraBase):
    def __init__(self, uid, name: str, base_save_dir: str, parent, carla_actor: carla.Sensor,
                 color_converter: carla.ColorConverter = None):
        super().__init__(uid, name, base_save_dir, parent, carla_actor, color_converter)