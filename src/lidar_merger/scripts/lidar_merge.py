#!/usr/bin/python3
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices
import yaml


class LidarMerger:
    def __init__(self, cfg, merged_topic='/merged_lidar/pointcloud'):
        rospy.init_node('lidar_merger', anonymous=True)
        
        # Subscribers
        rospy.Subscriber(cfg["lidar1"]["topic"], PointCloud2, self.callback_lidar1)
        rospy.Subscriber(cfg["lidar2"]["topic"], PointCloud2, self.callback_lidar2)

        # Publisher
        self.pub = rospy.Publisher(merged_topic, PointCloud2, queue_size=10)
        
        # Point cloud holders
        self.cloud1 = None
        self.cloud2 = None
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Define static transformation
        self.static_transform = TransformStamped()
        self.static_transform.header.stamp = rospy.Time.now()
        self.static_transform.header.frame_id = cfg["lidar1"]["frame_id"]
        self.static_transform.child_frame_id = cfg["lidar2"]["frame_id"]
        self.static_transform.transform.translation.x = cfg["lidar2"]["static_transform"]["translation"][0] # positive(left) or negative(right)
        self.static_transform.transform.translation.y = cfg["lidar2"]["static_transform"]["translation"][1] # positive(backward) or negative(forward)
        self.static_transform.transform.translation.z = cfg["lidar2"]["static_transform"]["translation"][2] # positive(up) or negative(down)
        self.static_transform.transform.rotation.x = cfg["lidar2"]["static_transform"]["rotation"][0]
        self.static_transform.transform.rotation.y = cfg["lidar2"]["static_transform"]["rotation"][1]
        self.static_transform.transform.rotation.z = cfg["lidar2"]["static_transform"]["rotation"][2]
        self.static_transform.transform.rotation.w = cfg["lidar2"]["static_transform"]["rotation"][3]


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

    def callback_lidar1(self, data):
        self.cloud1 = data

    def callback_lidar2(self, data):
        self.cloud2 = data

    def try_publish(self):
        if self.cloud1 is not None and self.cloud2 is not None:
            points1 = list(pc2.read_points(self.cloud1, skip_nans=True))
            points2 = list(pc2.read_points(self.cloud2, skip_nans=True))
            points2_transformed = self.transform_points([(p[0], p[1], p[2] if len(p) > 2 else 0) for p in points2], self.static_transform)

            merged_points = points1 + [tuple(p) + tuple(data[3:]) for p, data in zip(points2_transformed, points2)]
            merged_cloud = pc2.create_cloud(self.cloud1.header, self.cloud1.fields, merged_points)
            self.pub.publish(merged_cloud)
            self.cloud1 = None
            self.cloud2 = None

    def main(self):
        while not rospy.is_shutdown():
            if self.cloud1 and self.cloud2:
                self.try_publish()

if __name__ == '__main__':
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
    # print(config["lidar1"]["topic"])
    merger = LidarMerger(cfg=config)
    merger.main()
