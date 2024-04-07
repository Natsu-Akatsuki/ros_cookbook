import time
from pathlib import Path

import cv2
import yaml

from ampcl.calibration import calibration, object3d_kitti
from ampcl.calibration.calibration_kitti import KITTICalibration
from ampcl.io import load_pointcloud
from ampcl.ros import marker, publisher
from easydict import EasyDict
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from ampcl.ros import *
from ampcl.ros import __ROS_VERSION__

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from am_rviz_plugins_msgs.msg import BoundingBox, BoundingBoxArray


class Visualization(Node):
    def __init__(self):
        super().__init__("visualization")
        self.init_param()
        self.load_dataset()
        self.init_model()

    def init_param(self):
        cfg_file = "config/kitti.yaml"
        with open(cfg_file, 'r') as f:
            cfg = EasyDict(yaml.load(f, Loader=yaml.FullLoader))

        self.limit_range = cfg.AlgorithmParam.limit_range
        self.frame_id = cfg.ROSParam.frame_id

        if __ROS_VERSION__ == 1:
            # TODO：补充 ROS1 的接口
            pass
        if __ROS_VERSION__ == 2:
            latching_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

            self.pub_dict = {
                # marker
                "/box3d_marker": self.create_publisher(BoundingBoxArray, "/box3d_marker", latching_qos),
                "/distance_marker": self.create_publisher(MarkerArray, "/distance_marker", latching_qos),

                # pc（地面点，区域内的点云和区域外的点云）
                "/pc/ground": self.create_publisher(PointCloud2, "/pc/ground", latching_qos),
                "/pc/in_region": self.create_publisher(PointCloud2, "/pc/in_region", latching_qos),
                "/pc/out_region": self.create_publisher(PointCloud2, "/pc/out_region", latching_qos)
            }

            distance_marker = marker.create_distance_marker(frame_id=self.frame_id, distance_delta=10)
            self.pub_dict["/distance_marker"].publish(distance_marker)

            # Algorithm
            self.auto_update = cfg.AlgorithmParam.auto_update
            self.update_time = cfg.AlgorithmParam.update_time
            self.apply_fov_filter = cfg.AlgorithmParam.apply_fov_filter

            # Data
            dataset_dir = Path(cfg.DatasetParam.dataset_dir)
            self.img_dir = dataset_dir / Path(cfg.DatasetParam.img_dir)
            self.pc_dir = dataset_dir / cfg.DatasetParam.pc_dir
            self.gt_label_dir = dataset_dir / cfg.DatasetParam.gt_label_dir
            self.cal_dir = dataset_dir / cfg.DatasetParam.cal_dir
            self.split_file = dataset_dir / cfg.DatasetParam.split_file
            self.pred_label_dir = dataset_dir / cfg.DatasetParam.pred_label_dir

    def load_dataset(self):

        with open(self.split_file, 'r') as f:
            lines = f.readlines()

        pbar = tqdm(lines)
        for i, file_idx in enumerate(pbar):
            if (__ROS_VERSION__ == 1 and rospy.is_shutdown()) \
                    or (__ROS_VERSION__ == 2 and not rclpy.ok()):
                exit(0)
            # 读取图片、点云、标定外参数据
            file_idx = file_idx.strip()
            gt_label_path = "{}/{}.txt".format(self.gt_label_dir, file_idx)
            img_path = "{}/{}.png".format(self.img_dir, file_idx)
            pc_path = "{}/{}.bin".format(self.pc_dir, file_idx)
            cal_path = "{}/{}.txt".format(self.cal_dir, file_idx)
            pred_label_dir = "{}/{}.txt".format(self.pred_label_dir, file_idx)

            calib = KITTICalibration(cal_path)
            cal_info = calib.cal_info

            gt_infos = object3d_kitti.kitti_object_to_pcdet_object(gt_label_path, cal_info)
            pred_infos = object3d_kitti.kitti_object_to_pcdet_object(pred_label_dir, cal_info)

            img = cv2.imread(img_path)
            pc_np = load_pointcloud(pc_path)

            self.publish_result(pc_np, img, gt_infos, pred_infos, cal_info)
            pbar.set_description("Finish frame %s" % file_idx)
            if self.auto_update:
                time.sleep(self.update_time)
            else:
                input(f" press ↵ to continue...")

    def publish_result(self, pc_np, img, gt_info, pred_infos, cal_info):

        stamp = self.get_clock().now().to_msg()
        header = publisher.create_header(stamp, self.frame_id)

        # 只保留在相机视野内的点云
        if self.apply_fov_filter:
            _, _, mask = calibration.lidar_to_pixel(pc_np, cal_info, img_shape=img.shape[:2], use_mask=True)
            pc_np = pc_np[mask]

        now = self.get_clock().now().to_msg()
        box3d_marker_array = BoundingBoxArray()
        box3d_marker_array.header.stamp = now
        box3d_marker_array.header.frame_id = "velodyne"

        if pred_infos is not None:
            pred_box3d_lidar = pred_infos['box3d_lidar']

            for pred_box3d in pred_box3d_lidar:
                box3d_marker = BoundingBox()

                box3d_marker.header.stamp = now
                box3d_marker.header.frame_id = "velodyne"
                box3d_marker.label = 0
                box3d_marker.value = 0.0
                box3d_marker.attr = box3d_marker.TP

                q = Rotation.from_euler("ZYX", [pred_box3d[6], 0, 0]).as_quat()
                box3d_marker.pose.orientation.x = q[0]
                box3d_marker.pose.orientation.y = q[1]
                box3d_marker.pose.orientation.z = q[2]
                box3d_marker.pose.orientation.w = q[3]

                box3d_marker.pose.position.x = pred_box3d[0]
                box3d_marker.pose.position.y = pred_box3d[1]
                box3d_marker.pose.position.z = pred_box3d[2]

                box3d_marker.dimensions.x = pred_box3d[3]
                box3d_marker.dimensions.y = pred_box3d[4]
                box3d_marker.dimensions.z = pred_box3d[5]

                box3d_marker_array.boxes.append(box3d_marker)

        if gt_info is not None:
            gt_box3d_lidar = gt_info['box3d_lidar']

            for gt_box3d in gt_box3d_lidar:
                box3d_marker = BoundingBox()

                box3d_marker.header.stamp = now
                box3d_marker.header.frame_id = "velodyne"
                box3d_marker.label = 0
                box3d_marker.value = 0.0
                box3d_marker.attr = box3d_marker.GT

                q = Rotation.from_euler("ZYX", [gt_box3d[6], 0, 0]).as_quat()
                box3d_marker.pose.orientation.x = q[0]
                box3d_marker.pose.orientation.y = q[1]
                box3d_marker.pose.orientation.z = q[2]
                box3d_marker.pose.orientation.w = q[3]

                box3d_marker.pose.position.x = gt_box3d[0]
                box3d_marker.pose.position.y = gt_box3d[1]
                box3d_marker.pose.position.z = gt_box3d[2]

                box3d_marker.dimensions.x = gt_box3d[3]
                box3d_marker.dimensions.y = gt_box3d[4]
                box3d_marker.dimensions.z = gt_box3d[5]

                box3d_marker_array.boxes.append(box3d_marker)

            self.pub_dict["/box3d_marker"].publish(box3d_marker_array)

        publisher.publish_pc_by_range(self.pub_dict["/pc/in_region"],
                                      self.pub_dict["/pc/out_region"],
                                      pc_np,
                                      header,
                                      self.limit_range,
                                      field="xyzi",
                                      )


if __name__ == '__main__':
    if __ROS_VERSION__ == 1:
        Visualization(Node)
    elif __ROS_VERSION__ == 2:
        rclpy.init()
        node = Visualization()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
