import os
import subprocess
import time
from signal import SIGINT, SIGKILL

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from capella_ros_service_interfaces.srv import SwitchResolution
from capella_ros_service_interfaces.msg import RgbCameraResolution

LAUNCH_LOW = 'rgb_camera_back_640x480.launch.py'
LAUNCH_HIGH = 'rgb_camera_back_1280x1024.launch.py'
SHUTDOWN_TIMEOUT = 10.0

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
        self.current_procs = [self._spawn(LAUNCH_LOW)]

    def _spawn(self, launch_file):
        """Start a launch in its own session so the whole tree can be signalled.

        `ros2 launch` is only a wrapper: the process holding the V4L2 fd and the
        kernel-side capture buffers is the camera node underneath it. A new
        session makes the wrapper the leader of a process group containing both,
        so killpg still reaches the node after the wrapper exits and the node is
        reparented to PID 1.
        """
        return subprocess.Popen(
            ["ros2", "launch", "usb_cam", launch_file],
            start_new_session=True)

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
            self.current_procs = [self._spawn(LAUNCH_LOW)]
        elif self.mode == RgbCameraResolution.RESOLUTION_HIGH:
            self.current_procs = [self._spawn(LAUNCH_HIGH)]
        else:
            self._logger.info('rgb_camera_manager server transitioned to state %d' % RgbCameraResolution.RESOLUTION_IDLE)

        response.success = True
        return response

    def _signal_group(self, pgid, sig):
        """Signal a whole process group. Returns True if the group existed."""
        try:
            os.killpg(pgid, sig)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            self.get_logger().error('no permission to signal process group %d' % pgid)
            return False

    def _wait_stopped(self, proc, pgid, timeout):
        """Block until both the wrapper and every process in its group are gone."""
        deadline = time.monotonic() + timeout
        while True:
            # poll() also reaps the wrapper, so it cannot linger as a zombie
            # and keep the group probe alive.
            wrapper_done = proc.poll() is not None
            group_alive = True
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                group_alive = False
            except PermissionError:
                pass
            if wrapper_done and not group_alive:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def terminate(self, proc: subprocess.Popen, timeout: float = SHUTDOWN_TIMEOUT):
        """Stop a spawned launch and every process in its group, blocking until gone.

        Signalling the group rather than the `ros2 launch` wrapper is what
        actually reaches the camera node. The previous early return on
        `proc.poll() is not None` only observed the wrapper, so a node orphaned
        by an already-exited wrapper was never signalled at all and kept the
        camera fd and its kernel-side buffers open indefinitely.
        """
        if proc is None:
            return

        pgid = proc.pid  # start_new_session=True makes the child its own leader

        if not self._signal_group(pgid, SIGINT) and proc.poll() is not None:
            return

        if self._wait_stopped(proc, pgid, timeout):
            return

        self.get_logger().warn(
            'process group %d did not stop within %.1fs, sending SIGKILL' % (pgid, timeout))
        self._signal_group(pgid, SIGKILL)

        if not self._wait_stopped(proc, pgid, timeout):
            self.get_logger().error('process group %d is still present after SIGKILL' % pgid)

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