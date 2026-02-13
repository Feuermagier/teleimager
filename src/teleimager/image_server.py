# Copyright 2025 YuShu TECHNOLOGY CO.,LTD ("Unitree Robotics")
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import argparse
import glob
import cv2
from pathlib import Path
import logging_mp

# uvc will be imported when needed
import yaml
import time
import threading
import signal
import functools
import subprocess
import platform

from teleimager.cameras.base_camera import BaseCamera
from teleimager.cameras.isaac import IsaacSimCamera
from teleimager.cameras.opencv import OpenCVCamera
from teleimager.cameras.realsense import RealSenseCamera
# from teleimager.cameras.uvc import UVCCamera
from teleimager.cameras.zed import ZedCamera
from teleimager.network.webrtc_net import WebRTC_PublisherManager
from teleimager.network.zmq_net import ZMQ_PublisherManager, ZMQ_Responser


logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

# ========================================================
# cam_config_server.yaml path
# ========================================================

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "cam_config_server.yaml"
)
CONFIG_PATH = os.path.normpath(CONFIG_PATH)

# ========================================================
# certificate and key paths
# ========================================================
module_dir = Path(__file__).resolve().parent.parent.parent
default_cert = module_dir / "cert.pem"
default_key = module_dir / "key.pem"
env_cert = os.getenv("XR_TELEOP_CERT")
env_key = os.getenv("XR_TELEOP_KEY")
user_config_dir = Path.home() / ".config" / "xr_teleoperate"
user_cert = user_config_dir / "cert.pem"
user_key = user_config_dir / "key.pem"
CERT_PEM_PATH = Path(env_cert or (user_cert if user_cert.exists() else default_cert))
KEY_PEM_PATH = Path(env_key or (user_key if user_key.exists() else default_key))
CERT_PEM_PATH = CERT_PEM_PATH.resolve()
KEY_PEM_PATH = KEY_PEM_PATH.resolve()


# ========================================================
# UVC driver reload
# ========================================================
def reload_uvc_driver():
    try:
        subprocess.run("sudo modprobe -r uvcvideo", shell=True, check=True)
        time.sleep(1)
        subprocess.run("sudo modprobe uvcvideo debug=0", shell=True, check=True)
        time.sleep(1)
        logger_mp.info("UVC driver reloaded successfully.")
    except subprocess.CalledProcessError as e:
        logger_mp.error(f"Failed to reload driver: {e}")


# ========================================================
# camera finder and cameras
# ========================================================
class CameraFinder:
    """
    Discover connected cameras and their properties.
    vpath: /dev/videoX
    ppath: physical path in /sys/class/video4linux, e.g. /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0
    uid: USB unique ID, e.g. "001:002"
    dev_info: extra info from uvc
    sn: serial number of the camera
    """

    def __init__(self, realsense_enable=False, zed_enable=False, verbose=False):
        self.verbose = verbose
        # uvc
        # reload_uvc_driver()
        # import uvc

        # self.uvc_devices = uvc.device_list()
        # self.uid_map = {dev["uid"]: dev for dev in self.uvc_devices}

        # all video devices
        self.video_paths = self._list_video_paths()

        # realsense
        if realsense_enable:
            self.rs_serial_numbers = self._list_realsense_serial_numbers()
            self.rs_video_paths = self._list_realsense_video_paths()
            self.rs_rgb_video_paths = [
                p for p in self.rs_video_paths if self._is_like_rgb(p)
            ]
        else:
            self.rs_serial_numbers = []
            self.rs_video_paths = []
            self.rs_rgb_video_paths = []
        # zed
        if zed_enable:
            self.zed_serial_numbers = ZedCamera.find_all()
        else:
            self.zed_serial_numbers = []

        # # rgb & uvc
        # self.uvc_rgb_video_paths = self._list_uvc_rgb_video_paths()
        # self.uvc_rgb_video_ids = [
        #     int(v.replace("/dev/video", "")) for v in self.uvc_rgb_video_paths
        # ]
        # self.uvc_rgb_physical_paths = [
        #     self._get_ppath_from_vpath(v) for v in self.uvc_rgb_video_paths
        # ]
        # self.uvc_rgb_uids = [
        #     self._get_uid_from_ppath(p) for p in self.uvc_rgb_physical_paths
        # ]
        # self.uvc_rgb_dev_info = [self.uid_map.get(uid) for uid in self.uvc_rgb_uids]
        # self.uvc_rgb_serial_numbers = [
        #     dev_info.get("serialNumber") if dev_info else None
        #     for dev_info in self.uvc_rgb_dev_info
        # ]
        # # all uvc cameras
        # self.uvc_rgb_cameras = {}
        # for vpath, vid, ppath, uid, dev_info, sn in zip(
        #     self.uvc_rgb_video_paths,
        #     self.uvc_rgb_video_ids,
        #     self.uvc_rgb_physical_paths,
        #     self.uvc_rgb_uids,
        #     self.uvc_rgb_dev_info,
        #     self.uvc_rgb_serial_numbers,
        # ):
        #     self.uvc_rgb_cameras[vpath] = {
        #         "video_id": vid,
        #         "physical_path": ppath,
        #         "uid": uid,
        #         "dev_info": dev_info,
        #         "serial_number": sn,
        #     }
        if self.verbose:
            self.info()

    # utils
    def _list_video_paths(self):
        base = "/sys/class/video4linux/"
        if not os.path.exists(base):
            return []
        return [f"/dev/{x}" for x in sorted(os.listdir(base)) if x.startswith("video")]

    def _list_uvc_rgb_video_paths(self):
        return [
            p
            for p in self.video_paths
            if self._is_like_rgb(p) and p not in self.rs_video_paths
        ]

    def _list_realsense_video_paths(self):
        def _read_text(path):
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    return f.read().strip()
            except Exception:
                return None

        def _parent_usb_device_sysdir(video_sysdir):
            d = os.path.realpath(os.path.join(video_sysdir, "device"))
            for _ in range(10):
                if d is None or d == "/" or not os.path.isdir(d):
                    break
                id_vendor = _read_text(os.path.join(d, "idVendor"))
                id_product = _read_text(os.path.join(d, "idProduct"))
                if id_vendor and id_product:
                    return d
                d_next = os.path.dirname(d)
                if d_next == d:
                    break
                d = d_next
            return None

        ports = []
        for devnode in sorted(glob.glob("/dev/video*")):
            sysdir = f"/sys/class/video4linux/{os.path.basename(devnode)}"
            name = _read_text(os.path.join(sysdir, "name"))
            usb_dir = _parent_usb_device_sysdir(sysdir)
            vendor_id = (
                _read_text(os.path.join(usb_dir, "idVendor")) if usb_dir else None
            )

            # Match RealSense by name and Intel vendor ID
            if (
                name
                and "realsense" in name.lower()
                and (vendor_id or "").lower() in ("8086", "32902")
            ):
                ports.append(devnode)

        return ports

    def get_realsense_module(self) -> object:
        try:
            import pyrealsense2 as rs

            return rs
        except ImportError:
            arch = platform.machine()
            system = platform.system()
            print(f"[RealSense] Platform: {system} / {arch}")

            if system == "Linux" and arch.startswith("aarch64"):
                # Jetson NX / arm64
                msg = (
                    "[RealSense] pyrealsense2 not installed. please build from source:\n"
                    "    cd ~\n"
                    "    git clone https://github.com/IntelRealSense/librealsense.git\n"
                    "    cd librealsense\n"
                    "    git checkout v2.50.0\n"
                    "    mkdir build && cd build\n"
                    "    cmake .. -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3)\n"
                    "    make -j$(nproc)\n"
                    "    sudo make install\n"
                )
            else:
                # x86/x64
                msg = (
                    "[RealSense] pyrealsense2 not installed. You can try:\n"
                    "    pip install pyrealsense2\n"
                )
            raise RuntimeError(msg)

    def _list_realsense_serial_numbers(self):
        rs = self.get_realsense_module()
        ctx = rs.context()
        devices = ctx.query_devices()
        serials = []
        for dev in devices:
            try:
                serials.append(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                continue
        return serials

    def _get_ppath_from_vpath(self, video_path):
        sysfs_path = f"/sys/class/video4linux/{os.path.basename(video_path)}/device"
        return os.path.realpath(sysfs_path)

    def _get_uid_from_ppath(self, physical_path):
        def read_file(path):
            return open(path).read().strip() if os.path.exists(path) else None

        busnum_file = os.path.join(physical_path, "busnum")
        devnum_file = os.path.join(physical_path, "devnum")

        if not (os.path.exists(busnum_file) and os.path.exists(devnum_file)):
            parent = os.path.dirname(physical_path)
            busnum_file = os.path.join(parent, "busnum")
            devnum_file = os.path.join(parent, "devnum")

        if os.path.exists(busnum_file) and os.path.exists(devnum_file):
            bus = read_file(busnum_file)
            dev = read_file(devnum_file)
            return f"{bus}:{dev}"
        return None

    def _is_like_rgb(self, video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            return False
        ret, frame = cap.read()
        cap.release()
        return ret and frame is not None and frame.ndim == 3 and frame.shape[2] == 3

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def is_rs_serial_exist(self, serial_number):
        return str(serial_number) in self.rs_serial_numbers

    def is_vpath_exist(self, vpath):
        return vpath in self.video_paths

    def is_ppath_exist(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return True
        return False

    def get_uid_by_sn(self, serial_number):
        matches = [
            cam
            for cam in self.uvc_rgb_cameras.values()
            if cam.get("serial_number") == str(serial_number)
        ]
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(
                f"Multiple cameras found with serial number {serial_number}"
            )
        return matches[0].get("uid")

    def get_uid_by_ppath(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return cam.get("uid")
        return None

    def get_uid_by_vpath(self, video_path):
        cam = self.uvc_rgb_cameras.get(video_path)
        if cam:
            return cam.get("uid")
        return None

    def get_vpath_by_sn(self, serial_number):
        matches = []
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("serial_number") == str(serial_number):
                vpath = f"/dev/video{cam.get('video_id')}"
                matches.append(vpath)
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(
                f"Multiple video devices found for serial number {serial_number}: {matches}. "
            )
        return matches[0]

    def get_vpath_by_ppath(self, physical_path):
        base = "/sys/class/video4linux/"
        matches = []
        for v in os.listdir(base):
            sys_path = os.path.realpath(os.path.join(base, v, "device"))
            if sys_path == physical_path:
                vpath = f"/dev/{v}"
                if self._is_like_rgb(vpath):
                    matches.append(vpath)
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(
                f"Multiple video devices found for physical path {physical_path}: {matches}. "
            )
        return matches[0]

    def info(self):
        logger_mp.info(
            "======================= Camera Discovery Start =================================="
        )
        logger_mp.info("Found video devices: %s", self.video_paths)
        # logger_mp.info("Found RGB video devices: %s", self.uvc_rgb_video_paths)

        if self.rs_serial_numbers:
            logger_mp.info(
                "----------------------- Realsense Cameras ----------------------------------"
            )
            logger_mp.info(f"RealSense serial numbers: {self.rs_serial_numbers}")
            logger_mp.info(f"RealSense video paths: {self.rs_video_paths}")
            logger_mp.info(f"RealSense RGB-like video paths: {self.rs_rgb_video_paths}")

        if self.zed_serial_numbers:
            logger_mp.info(
                "----------------------- Zed Cameras ----------------------------------"
            )
            logger_mp.info(f"Zed serial numbers: {self.zed_serial_numbers}")

        # for idx, (vpath, cam) in enumerate(self.uvc_rgb_cameras.items(), start=1):
        #     logger_mp.info(
        #         "----------------------- OpenCV / UVC Camera %d -----------------------------",
        #         idx,
        #     )
        #     logger_mp.info("video_path    : %s", vpath)
        #     logger_mp.info("video_id      : %s", cam.get("video_id"))
        #     logger_mp.info("serial_number : %s", cam.get("serial_number") or "unknown")
        #     logger_mp.info("physical_path : %s", cam.get("physical_path"))
        #     logger_mp.info("extra_info:")

        #     dev_info = cam.get("dev_info")
        #     uid = cam.get("uid")

        #     if dev_info:
        #         for k, v in dev_info.items():
        #             logger_mp.info("    %s: %s", k, v)
        #         try:
        #             import uvc

        #             cap = uvc.Capture(uid)
        #             for fmt in cap.available_modes:
        #                 logger_mp.info(
        #                     "    format: %dx%d@%d %s",
        #                     fmt.height,
        #                     fmt.width,
        #                     fmt.fps,
        #                     fmt.format_name,
        #                 )
        #             cap.close()
        #             cap = None
        #         except Exception as e:
        #             logger_mp.warning("    failed to get formats: %s", e)
        #     else:
        #         logger_mp.info("    no uvc extra info available")

        logger_mp.info(
            "=========================== Camera Discovery End ================================"
        )


# ========================================================
# image server
# ========================================================
class ImageServer:
    def __init__(
        self,
        cam_config,
        realsense_enable=False,
        camera_finder_verbose=False,
        isaacsim_enable=False,
    ):
        self._cam_config = cam_config
        self._realsense_enable = realsense_enable
        self._isaacsim_enable = isaacsim_enable
        self._stop_event = threading.Event()
        self._cameras: dict[str, BaseCamera] = {}
        if not self._isaacsim_enable:
            self._cam_finder = CameraFinder(realsense_enable, camera_finder_verbose)
        self._responser = ZMQ_Responser(self._cam_config)
        self._zmq_publisher_manager = ZMQ_PublisherManager.get_instance()
        self._webrtc_publisher_manager = WebRTC_PublisherManager(CERT_PEM_PATH, KEY_PEM_PATH)
        self._publisher_threads = []  # keep references for graceful join

        try:
            # Load cameras from self.cam_config
            for cam_topic, cam_cfg in self._cam_config.items():
                if not cam_cfg.get("enable_zmq", False) and not cam_cfg.get(
                    "enable_webrtc", False
                ):
                    continue

                enable_zmq = cam_cfg.get("enable_zmq", False)
                zmq_port = cam_cfg.get("zmq_port", None)
                enable_webrtc = cam_cfg.get("enable_webrtc", False)
                webrtc_port = cam_cfg.get("webrtc_port", None)
                webrtc_codec = cam_cfg.get("webrtc_codec", None)
                cam_type = cam_cfg.get("type", "uvc").lower()
                if self._isaacsim_enable and cam_type != "isaacsim":
                    cam_type = "isaacsim"
                img_shape = cam_cfg.get("image_shape", None)
                fps = cam_cfg.get("fps", 30)
                video_id = cam_cfg.get("video_id", "0")
                video_path = f"/dev/video{video_id}" if video_id else None
                physical_path = (
                    str(cam_cfg.get("physical_path"))
                    if cam_cfg.get("physical_path")
                    else None
                )
                serial_number = (
                    str(cam_cfg.get("serial_number"))
                    if cam_cfg.get("serial_number")
                    else None
                )

                if cam_type == "opencv":
                    if physical_path is not None:
                        vpath = self._cam_finder.get_vpath_by_ppath(physical_path)
                        if vpath is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(
                                f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with physical path {physical_path}"
                            )
                        else:
                            self._cameras[cam_topic] = OpenCVCamera(
                                cam_topic,
                                vpath,
                                img_shape,
                                fps,
                                enable_zmq,
                                zmq_port,
                                enable_webrtc,
                                webrtc_port,
                                webrtc_codec,
                            )
                            continue

                    if serial_number is not None:
                        vpath = self._cam_finder.get_vpath_by_sn(serial_number)
                        if vpath is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(
                                f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with serial number {serial_number}"
                            )
                        else:
                            self._cameras[cam_topic] = OpenCVCamera(
                                cam_topic,
                                vpath,
                                img_shape,
                                fps,
                                enable_zmq,
                                zmq_port,
                                enable_webrtc,
                                webrtc_port,
                                webrtc_codec,
                            )
                        # once you specify either `physical_path` or `serial_number`, the system will no longer fall back to searching by `video_id`.
                        # ——— even if no camera matches the given path/serial.
                        continue

                    if not self._cam_finder.is_vpath_exist(video_path):
                        self._cameras[cam_topic] = None
                        logger_mp.error(
                            f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with video_id {video_id}"
                        )
                    else:
                        self._cameras[cam_topic] = OpenCVCamera(
                            cam_topic,
                            video_path,
                            img_shape,
                            fps,
                            enable_zmq,
                            zmq_port,
                            enable_webrtc,
                            webrtc_port,
                            webrtc_codec,
                        )

                elif cam_type == "realsense":
                    if not self._realsense_enable:
                        self._cameras[cam_topic] = None
                        logger_mp.error(
                            f"[Image Server] Please start image server with the '--rs' flag to support Realsense {cam_topic}."
                        )
                    elif not self._cam_finder.is_rs_serial_exist(serial_number):
                        self._cameras[cam_topic] = None
                        logger_mp.error(
                            f"[Image Server] Cannot find RealSenseCamera for {cam_topic}"
                        )
                    else:
                        self._cameras[cam_topic] = RealSenseCamera(
                            cam_topic,
                            serial_number,
                            img_shape,
                            fps,
                            enable_zmq,
                            zmq_port,
                            enable_webrtc,
                            webrtc_port,
                            webrtc_codec,
                        )
                elif cam_type == "zed":
                    self._cameras[cam_topic] = ZedCamera(
                        cam_topic,
                        serial_number,
                        img_shape,
                        fps,
                        enable_zmq,
                        zmq_port,
                        enable_webrtc,
                        webrtc_port,
                        webrtc_codec,
                    )

                elif cam_type == "uvc":
                    uid = None
                    if physical_path is not None:
                        uid = self._cam_finder.get_uid_by_ppath(physical_path)
                        if uid is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(
                                f"[Image Server] Cannot find UVCCamera for {cam_topic} with physical path {physical_path}"
                            )
                        else:
                            self._cameras[cam_topic] = UVCCamera(
                                cam_topic,
                                uid,
                                img_shape,
                                fps,
                                enable_zmq,
                                zmq_port,
                                enable_webrtc,
                                webrtc_port,
                                webrtc_codec,
                            )
                            continue

                    if serial_number is not None:
                        uid = self._cam_finder.get_uid_by_sn(serial_number)
                        if uid is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(
                                f"[Image Server] Cannot find UVCCamera for {cam_topic} with serial number {serial_number}"
                            )
                        else:
                            self._cameras[cam_topic] = UVCCamera(
                                cam_topic,
                                uid,
                                img_shape,
                                fps,
                                enable_zmq,
                                zmq_port,
                                enable_webrtc,
                                webrtc_port,
                                webrtc_codec,
                            )
                        # once you specify either `physical_path` or `serial_number`, the system will no longer fall back to searching by `video_id`.
                        # ——— even if no camera matches the given path/serial.
                        continue
                elif cam_type == "isaacsim":
                    # Check if binocular mode is enabled
                    binocular = cam_cfg.get("binocular", False)

                    # For IsaacSim cameras, determine image source based on camera topic and binocular setting
                    if binocular:
                        # Binocular cameras (like head) need to read left+right and concatenate
                        image_source = "head"  # Special marker for binocular
                    else:
                        # Monocular cameras read their specific source
                        if "left" in cam_topic.lower():
                            image_source = "left"
                        elif "right" in cam_topic.lower():
                            image_source = "right"
                        else:
                            image_source = "head"  # fallback

                    self._cameras[cam_topic] = IsaacSimCamera(
                        cam_topic,
                        img_shape,
                        fps,
                        enable_zmq,
                        zmq_port,
                        enable_webrtc,
                        webrtc_port,
                        webrtc_codec,
                        image_source=image_source,
                        binocular=binocular,
                    )
                else:
                    logger_mp.error(
                        f"[Image Server] Unknown camera type {cam_type} for {cam_topic}, skipping..."
                    )
                    continue
        except Exception as e:
            logger_mp.error(f"[Image Server] Initialization failed: {e}")
            self._clean_up()
            raise

        logger_mp.info(
            "[Image Server] Image server has started, waiting for client connections..."
        )

    def _update_frames(self, cam_topic: str, camera: BaseCamera):
        try:
            webrtc_codec = camera.get_webrtc_codec()
            interval = 1.0 / camera.get_fps()
            next_frame_time = time.monotonic()
            while not self._stop_event.is_set():
                try:
                    camera._update_frame()
                    frame = camera.get_bgr_frame()

                    # directly push frames to the publisher threads, don't use a separate background thread for this
                    if camera.enable_webrtc:
                        self._webrtc_publisher_manager.publish(
                            frame, camera.get_webrtc_port(), codec_pref=webrtc_codec
                        )

                    if camera.enable_zmq:
                        self._zmq_publisher_manager.publish(
                            frame, camera.get_zmq_port()
                        )
                except Exception as e:
                    logger_mp.error(
                        f"[Image Server] Error updating frame for {cam_topic} camera"
                    )
                    self._stop_event.set()
                    raise

                # usually the camera ensures constant fps, but to be sure we sleep here again
                # TODO this may be problematic if we wait slightly too long and jump into the next frame
                # next_frame_time += interval
                # sleep_time = next_frame_time - time.monotonic()
                # if sleep_time > 0:
                #     time.sleep(sleep_time)
                # else:
                #     next_frame_time = time.monotonic()
        except Exception as e:
            logger_mp.error(
                f"[Image Server] Failed to update frames for {cam_topic} camera: {e}"
            )
            self._stop_event.set()
            raise

    def _clean_up(self):
        self._responser.stop()
        for t in self._publisher_threads:
            if t.is_alive():
                t.join(timeout=1.0)
        self._publisher_threads.clear()

        try:
            self._zmq_publisher_manager.close()
        except Exception:
            pass
        try:
            self._webrtc_publisher_manager.close()
        except Exception:
            pass

        for cam in self._cameras.values():
            if cam:
                try:
                    cam.release()
                except Exception as e:
                    logger_mp.error(
                        f"[Image Server] Error releasing camera {cam._cam_topic}: {e}"
                    )
        logger_mp.info("[Image Server] Clean up completed. Server stopped.")

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def start(self):
        for camera_topic, camera in self._cameras.items():
            if camera is None:
                logger_mp.error(
                    f"[Image Server] Camera {camera_topic} failed to initialize previously, cannot start."
                )
                self._stop_event.set()
                self._clean_up()
                return
            t = threading.Thread(
                target=self._update_frames, args=(camera_topic, camera), daemon=True
            )
            t.start()
            self._publisher_threads.append(t)
        if self._isaacsim_enable:
            time.sleep(2.0)  # wait a bit for IsaacSim shared memory to be ready

        for camera_topic, camera in self._cameras.items():
            # Use longer timeout for IsaacSim cameras since they need to wait for shared memory data
            if self._isaacsim_enable:
                timeout = 15.0
            else:
                timeout = 5.0
            ready = camera.wait_until_ready(timeout=timeout)
            if not ready:
                logger_mp.error(
                    f"[Image Server] {camera_topic} ready timeout after {timeout}s."
                )
                self._stop_event.set()
                self._clean_up()
            logger_mp.info(f"[Image Server] {camera_topic} is ready.")

    def wait(self):
        self._stop_event.wait()
        self._clean_up()

    def stop(self):
        self._stop_event.set()


# ========================================================
# utility functions
# ========================================================
def signal_handler(server, signum, frame):
    logger_mp.info(
        f"[Image Server] Received signal {signum}, initiating graceful shutdown..."
    )
    server.stop()


def set_performance_mode(cores=[0, 1, 2]):
    import psutil

    try:
        p = psutil.Process(os.getpid())

        # Set CPU affinity for the process and all its threads
        p.cpu_affinity(cores)
        logger_mp.info(f"[Performance] CPU Affinity locked to: {cores}")

    except psutil.AccessDenied:
        logger_mp.warning(
            "[Performance] Access Denied: Run as sudo for full optimization"
        )
    except Exception as e:
        logger_mp.error(f"[Performance] Error: {e}")


def run_isaacsim_server():
    # Load config file, start image server
    try:
        with open(CONFIG_PATH, "r") as f:
            cam_config = yaml.safe_load(f)
    except Exception as e:
        logger_mp.error(f"Failed to load configuration file at {CONFIG_PATH}: {e}")
        exit(1)
    # start image server
    server = ImageServer(
        cam_config,
        realsense_enable=False,
        camera_finder_verbose=False,
        isaacsim_enable=True,
    )
    server.start()
    return server


def main():
    logger_mp.info(
        "\n====================== Image Server Startup Guide ======================\n"
        "Please first read this repo's README.md to learn how to configure and use the teleimager.\n"
        "To discover connected cameras, run the following command:\n"
        "\n"
        "    teleimager-server --cf\n"
        "\n"
        "The '--cf' flag means 'camera find'.\n"
        "This will list all detected cameras and their details (video paths, serial numbers and physical path etc.).\n"
        "Use that information to fill in your 'cam_config_server.yaml' file.\n"
        "Once configured, you can start the image server with:\n"
        "\n"
        "    teleimager-server\n"
        "\n"
        "Note:\n"
        " - If you have RealSense cameras, add the '--rs' flag to enable RealSense support.\n"
        " - Make sure you have proper permissions to access the camera devices (e.g., run with sudo or set udev rules).\n"
        "=========================================================================="
    )

    # command line args
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cf",
        action="store_true",
        help="Enable camera found mode, print all connected cameras info",
    )
    parser.add_argument(
        "--rs", action="store_true", help="Enable RealSense camera mode."
    )
    parser.add_argument("--zed", action="store_true", help="Enable Zed camera mode.")
    parser.add_argument(
        "--no-affinity",
        action="store_false",
        dest="affinity",
        help="Disable CPU affinity setting for performance optimization.",
    )
    args = parser.parse_args()

    if args.affinity:
        set_performance_mode(cores=[0, 1, 2])

    # if enable camera finder mode, just print cameras info and exit
    if args.cf:
        CameraFinder(realsense_enable=args.rs, zed_enable=args.zed, verbose=True)
        exit(0)

    # Load config file, start image server
    try:
        with open(CONFIG_PATH, "r") as f:
            cam_config = yaml.safe_load(f)
    except Exception as e:
        logger_mp.error(f"Failed to load configuration file at {CONFIG_PATH}: {e}")
        exit(1)

    # start image server
    server = ImageServer(
        cam_config, realsense_enable=args.rs, camera_finder_verbose=False
    )
    server.start()

    # graceful shutdown handling
    signal.signal(signal.SIGINT, functools.partial(signal_handler, server))
    signal.signal(signal.SIGTERM, functools.partial(signal_handler, server))

    logger_mp.info("[Image Server] Running... Press Ctrl+C to exit.")
    server.wait()

    # usbhub plugout may cause block process exit, no better solution for now
    time.sleep(0.5)
    os.killpg(os.getpgrp(), 9)


if __name__ == "__main__":
    main()
