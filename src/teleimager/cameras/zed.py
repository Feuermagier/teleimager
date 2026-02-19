import logging_mp
import time
import numpy as np

from teleimager.cameras.base_camera import BaseCamera

logger_mp = logging_mp.getLogger(__name__)


class ZedCamera(BaseCamera):
    def __init__(
        self,
        cam_topic,
        serial_number,
        img_shape,
        fps,
        enable_zmq=True,
        zmq_port=55555,
        enable_webrtc=False,
        webrtc_port=66666,
        webrtc_codec=None,
        enable_depth=False,
    ):

        self.sl = ZedCamera._try_import()
        super().__init__(
            cam_topic,
            img_shape,
            fps,
            enable_zmq,
            zmq_port,
            enable_webrtc,
            webrtc_port,
            webrtc_codec,
        )

        self.serial_number = serial_number
        self.name = f"Zed ({serial_number})"
        self.fps = fps
        self.n_warm_start = 5
        self.reconnect_attempts = 3

        self.resolution = self.sl.RESOLUTION["HD720"]
        self.depth_mode = self.sl.DEPTH_MODE["NONE"]

        self._connect()

        logger_mp.info("Zed initialized")

    def _connect(self):
        logger_mp.info(f"Connecting to Zed {self.name}")
        for i in range(self.reconnect_attempts):
            try:
                self._do_connect()
                logger_mp.info(f"Connection to Zed {self.name} successful.")
                return
            except Exception:
                if i == self.reconnect_attempts - 1:
                    logger_mp.error(
                        f'Failed to connect to Zed "{self.name}" after {self.reconnect_attempts} attempts.'
                    )
                    raise
                logger_mp.exception(
                    f"Attempt {i + 1} to connect to Zed {self.name} (serial no. {self.serial_number}) failed."
                )
                time.sleep(1)  # wait before retrying

    def _do_connect(self):
        self.zed = self.sl.Camera()

        init_params = self.sl.InitParameters()
        init_params.set_from_serial_number(int(self.serial_number))
        init_params.camera_resolution = self.resolution
        init_params.camera_fps = self.fps
        init_params.depth_mode = self.depth_mode
        init_params.coordinate_units = self.sl.UNIT.METER

        err = self.zed.open(init_params)
        if err != self.sl.ERROR_CODE.SUCCESS:
            logger_mp.error(f"Failed to open ZED camera: {err}")
            raise RuntimeError(f"Failed to open ZED camera: {err}")

        # fix white balance and exposure
        self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 5500)
        self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.GAIN, 1)
        self.zed.set_camera_settings(self.sl.VIDEO_SETTINGS.EXPOSURE, 14)

        self.image_left = self.sl.Mat()
        self.image_right = self.sl.Mat()
        self.image_both = self.sl.Mat()
        self.depth = self.sl.Mat()
        self.runtime_parameters = self.sl.RuntimeParameters()

        for _ in range(self.n_warm_start):
            self._retrieve_images()

    def _try_import():
        try:
            import pyzed.sl as sl

            return sl
        except Exception as e:
            raise ImportError("pyzed is probably not installed") from e

    def _update_frame(self):
        self._retrieve_images()

        t = time.perf_counter()
        full_frame = np.copy(self.image_both.get_data())[..., :3]

        if self.enable_webrtc:
            self._webrtc_buffer.write((full_frame, t))

        if not self._ready.is_set():
            self._ready.set()

    def _retrieve_images(self):
        if self.zed is None:
            raise RuntimeError(f"Not connected to {self.name}")

        status = self.zed.grab(self.runtime_parameters)
        if status == self.sl.ERROR_CODE.SUCCESS:
            # self.zed.retrieve_image(self.image_left, self.sl.VIEW.LEFT)
            # self.zed.retrieve_image(self.image_right, self.sl.VIEW.RIGHT)
            self.zed.retrieve_image(self.image_both, self.sl.VIEW.SIDE_BY_SIDE)
            # self.zed.retrieve_measure(self.depth, self.sl.MEASURE.DEPTH)
            # self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        else:
            raise RuntimeError(f"Failed to retrieve images from {self.name}: {status}")

    def get_depth_frame(self):
        if self._latest_depth is None:
            return None
        return self._latest_depth.tobytes()

    def release(self):
        self.zed.close()
        logger_mp.info(f"[Zed] Released {self._cam_topic}")

    def find_all():
        sl = ZedCamera._try_import()
        devices = sl.Camera.get_device_list()
        return [d.serial_number for d in devices]
