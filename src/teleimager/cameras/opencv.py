import logging_mp
import cv2


from teleimager.cameras.base_camera import BaseCamera

logger_mp = logging_mp.getLogger(__name__, level=logging_mp.INFO)


class OpenCVCamera(BaseCamera):
    def __init__(
        self,
        cam_topic,
        video_path,
        img_shape,
        fps,
        enable_zmq=True,
        zmq_port=55555,
        enable_webrtc=False,
        webrtc_port=66666,
        webrtc_codec=None,
    ):
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
        self._video_path = video_path

        self.cap = cv2.VideoCapture(self._video_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._img_shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._img_shape[1])
        self.cap.set(cv2.CAP_PROP_FPS, self._fps)

        # Test if the camera can read frames
        if not self._can_read_frame():
            self.release()
            raise RuntimeError(
                f"[OpenCVCamera] Camera {self._cam_topic} failed to initialize or read frames."
            )
        else:
            logger_mp.info(str(self))

    def __str__(self):
        return (
            f"[OpenCVCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def _can_read_frame(self):
        success, _ = self.cap.read()
        return success

    def _update_frame(self):
        if self.cap is not None:
            ret, bgr_numpy = self.cap.read()
            if ret:
                self._webrtc_buffer.write(bgr_numpy)

                if not self._ready.is_set():
                    self._ready.set()
            else:
                raise RuntimeError

    def release(self):
        self.cap.release()
        self.cap = None
        logger_mp.info(f"[OpenCVCamera] Released {self._cam_topic}")
