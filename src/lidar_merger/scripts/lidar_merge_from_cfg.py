#!/usr/bin/python3
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices
import yaml
from functools import partial

class LidarMerger:
    def __init__(self, cfg, merged_topic='/merged_lidar/pointcloud'):
        rospy.init_node('lidar_merger', anonymous=True)
        
        self.lidars_count = len(cfg.keys())

        # Subscribers
        for key in cfg.keys():
            rospy.Subscriber(cfg[key]["topic"], PointCloud2,  partial(self.callback_lidar, key))

        # Publisher
        self.pub = rospy.Publisher(merged_topic, PointCloud2, queue_size=10)
        
        # Point cloud holder for each lidar
        self.clouds = {key: None for key in cfg.keys()}
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Define transformation for each lidar
        self.static_transforms = {}

        for key in list(cfg.keys())[1:]:
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = cfg[key]["frame_id"]
            static_transform.child_frame_id = cfg[key]["frame_id"]
            static_transform.transform.translation.x = cfg[key]["static_transform"]["translation"][0]
            static_transform.transform.translation.y = cfg[key]["static_transform"]["translation"][1]
            static_transform.transform.translation.z = cfg[key]["static_transform"]["translation"][2]
            static_transform.transform.rotation.x = cfg[key]["static_transform"]["rotation"][0]
            static_transform.transform.rotation.y = cfg[key]["static_transform"]["rotation"][1]
            static_transform.transform.rotation.z = cfg[key]["static_transform"]["rotation"][2]
            static_transform.transform.rotation.w = cfg[key]["static_transform"]["rotation"][3]
            self.static_transforms[key] = static_transform
        # print(self.static_transforms)

    def transform_points(self, points, transform):
        T = concatenate_matrices(translation_matrix([transform.transform.translation.x,
                                                    transform.transform.translation.y,
                                                    transform.transform.translation.z]),
                                 quaternion_matrix([transform.transform.rotation.x,
                                                    transform.transform.rotation.y,
                                                    transform.transform.rotation.z,
                                                    transform.transform.rotation.w]))
        points_hom = np.ones((4, len(points)))
        points_hom[:3, :] = np.array(points).T
        points_transformed = np.dot(T, points_hom)[:3, :].T
        return points_transformed

    def callback_lidar(self, key, data):
        self.clouds[key] = data
        # print(f"Received point cloud from {key}")

    def try_publish(self):
        if all([self.clouds[key] is not None for key in self.clouds.keys()]):
            points = {}
            for key in self.clouds.keys():
                points[key] = list(pc2.read_points(self.clouds[key], skip_nans=True))
            
            points_transformed = {}
            for key in list(self.clouds.keys())[1:]:
                points_transformed[key] = self.transform_points([(p[0], p[1], p[2] if len(p) > 2 else 0) for p in points[key]], self.static_transforms[key])

            merged_points = points[list(self.clouds.keys())[0]]

            for key in list(self.clouds.keys())[1:]:
                merged_points = merged_points + [tuple(p) + tuple(data[3:]) for p, data in zip(points_transformed[key], points[key])]

            merged_cloud = pc2.create_cloud(self.clouds[list(self.clouds.keys())[0]].header, self.clouds[list(self.clouds.keys())[0]].fields, merged_points)

            self.pub.publish(merged_cloud)
            self.clouds = {key: None for key in self.clouds.keys()}
            # print("Published merged point cloud")

    def main(self):
        while not rospy.is_shutdown():
            if self.clouds["lidar1"]:
                self.try_publish()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Merge point clouds from multiple lidars')
    parser.add_argument('--config', type=str, default='src/lidar_merger/config/lidar_merger.yaml', help='Path to config file')
    args = parser.parse_args()

    try:
        print(args.config)
        config = yaml.load(open(args.config), Loader=yaml.FullLoader)
    except:
        print("Error loading config file. Using default values.")
        config = {
        'lidar1': {
            'topic': '/hesai/pandar',
            'frame_id': 'lidar1',
            'static_transform': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 0.0]
            }
        },
        'lidar2': {
            'topic': '/pandar163/pandar',
            'frame_id': 'lidar2',
            'static_transform': {
                'translation': [1.0, 2.5, -0.4],
                'rotation': [0.0, 0.0, 0.7071, 0.7071]
            }
        },
    }

    merger = LidarMerger(cfg=config)
    merger.main()


###################
# Pandar32 config #
###################
#   'lidar6': {
#     'topic': '/pandar32/pandar',
#     'frame_id': 'lidar6',
#     'static_transform': {
#         'translation': [0.0, 0.0, 1.4],
#         'rotation': [0.0, 0.0, 0.0, 0]
#     }
#   },