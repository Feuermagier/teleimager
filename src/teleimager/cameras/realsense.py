import logging_mp
import numpy as np
import cv2


from teleimager.base_camera import BaseCamera

logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

class RealSenseCamera(BaseCamera):
    def __init__(self, cam_topic, serial_number, img_shape, fps, 
                 enable_zmq=True, zmq_port = 55555, enable_webrtc=False, webrtc_port=66666, webrtc_codec=None, enable_depth=False):
        rs = self.check_pyrealsense2_install()
        super().__init__(cam_topic, img_shape, fps, enable_zmq, zmq_port, enable_webrtc, webrtc_port, webrtc_codec)
        self._serial_number = serial_number
        self._enable_depth = enable_depth
        self._latest_depth = None
        try:
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self._serial_number)

            config.enable_stream(rs.stream.color, self._img_shape[1], self._img_shape[0], rs.format.bgr8, self._fps)
            if self._enable_depth:
                config.enable_stream(rs.stream.depth, self._img_shape[1], self._img_shape[0], rs.format.z16, self._fps)

            profile = self.pipeline.start(config)
            self._device = profile.get_device()
            if self._device is None:
                logger_mp.error('[RealSenseCamera] pipe_profile.get_device() is None .')
            if self._enable_depth:
                assert self._device is not None
                depth_sensor = self._device.first_depth_sensor()
                self.g_depth_scale = depth_sensor.get_depth_scale()

            self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            logger_mp.info(str(self))
        except Exception as e:
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
            raise e
            raise RuntimeError(f"[RealSenseCamera] Failed to initialize RealSense camera {self._serial_number}: {e}")

    def __str__(self):
        return (
            f"[RealSenseCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS.\n"
            f"ZMQ: {'enabled, zmq_port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc_port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def check_pyrealsense2_install(self):
        try:
            import pyrealsense2 as rs
            return rs
        except Exception as e:
            raise ImportError(
                "pyrealsense2 not installed. Install Intel RealSense SDK and pyrealsense2 Python bindings."
            ) from e
    
    def _update_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None

        if self._enable_depth:   
            depth_frame = aligned_frames.get_depth_frame()
            if depth_frame:
                self._latest_depth = np.asanyarray(depth_frame.get_data())
            else:
                self._latest_depth = None

        bgr_numpy = np.asanyarray(color_frame.get_data())
        self._webrtc_buffer.write(bgr_numpy)
        
        if not self._ready.is_set():
            self._ready.set()
    
    def get_depth_frame(self):
        if self._latest_depth is None:
            return None
        return self._latest_depth.tobytes()

    def release(self):
        try:
            if hasattr(self.pipeline, "stop") and getattr(self.pipeline, "_running", False):
                try:
                    self.pipeline.stop()
                except Exception as e:
                    logger_mp.warning(f"[RealSenseCamera] pipeline.stop() failed: {e}")
        except Exception:
            pass
        self.pipeline = None
        logger_mp.info(f"[RealSenseCamera] Released {self._cam_topic}")
