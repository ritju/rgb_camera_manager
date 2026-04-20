import os
from signal import SIGINT

import psutil
import rclpy
import subprocess

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from capella_ros_service_interfaces.srv import SwitchResolution
from capella_ros_service_interfaces.msg import RgbCameraResolution

class NodeManagerService(Node):

    def __init__(self):
        super().__init__('rgb_camera_manager_server')

        # Track all processes we spawn (e.g., multiple ros2 launch commands)
        self.current_procs = []

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.status_publisher = self.create_publisher(RgbCameraResolution, 
            'rgb_camera_manager_server/resolution_mode', qos_profile = latching_qos)
        
        self.switch_resolution_srv = self.create_service(
            SwitchResolution, 'rgb_camera_manager_server/switch_resolution', self.switch_resolution)        

        self.set_status(RgbCameraResolution.RESOLUTION_IDLE)
        
        # 服务启动时初始化启动低分辨率
        self.set_status(RgbCameraResolution.RESOLUTION_LOW)
        self.current_procs = [
            subprocess.Popen(
                ["ros2", "launch", "usb_cam", "rgb_camera_back_640x480.launch.py"]
            )
        ]

    def set_status(self, newStatus):
        self.mode = newStatus

        msg = RgbCameraResolution()
        msg.resolution_mode = self.mode
        self.status_publisher.publish(msg)

        self._logger.info('rgb_camera_manager server transitioned to state %d' % newStatus)

    def switch_resolution(self, request:SwitchResolution.Request, response:SwitchResolution.Response):
        status_str = 'idle'
        if request.resolution_mode == SwitchResolution.Request.RESOLUTION_IDLE:
            status_str = 'idle'
        elif request.resolution_mode == SwitchResolution.Request.RESOLUTION_LOW:
            status_str = 'low'
        elif request.resolution_mode == SwitchResolution.Request.RESOLUTION_HIGH:
            status_str = 'high'
        else:
            status_str = 'idle'
        
        self.get_logger().info(f"Received a request for resolution {status_str}")
        
        response.resolution_mode = request.resolution_mode

        if self.mode == request.resolution_mode:
            self.get_logger().info(f"Current resolution is already {status_str}")
            response.success = True
            return response

        self.set_status(request.resolution_mode)
        self.stop_current()
        
        # Run both launches concurrently as separate processes.
        if self.mode == RgbCameraResolution.RESOLUTION_LOW:
            self.current_procs = [
                subprocess.Popen(
                    ["ros2", "launch", "usb_cam", "rgb_camera_back_640x480.launch.py"]
                )
            ]
        elif self.mode == RgbCameraResolution.RESOLUTION_HIGH:
            self.current_procs = [
                subprocess.Popen(
                    ["ros2", "launch", "usb_cam", "rgb_camera_back_1280x1024.launch.py"]
                )
            ]
        else:
            self._logger.info('rgb_camera_manager server transitioned to state %d' % RgbCameraResolution.RESOLUTION_IDLE)

        response.success = True
        return response

    def terminate(self, proc: subprocess.Popen):
        """Gracefully stop a spawned process and all its children."""
        if proc is None:
            return
        if proc.poll() is not None:
            return
        try:
            parent = psutil.Process(proc.pid)
        except psutil.NoSuchProcess:
            return

        for child in parent.children(recursive=True):
            try:
                child.send_signal(SIGINT)
            except psutil.NoSuchProcess:
                pass
        try:
            parent.send_signal(SIGINT)
        except psutil.NoSuchProcess:
            pass

    def stop_current(self):
        if self.current_procs:
            for proc in self.current_procs:
                self.terminate(proc)
            self.current_procs = []

def main():
    rclpy.init()

    service = NodeManagerService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()