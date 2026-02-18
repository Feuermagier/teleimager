import logging_mp


from teleimager.cameras.base_camera import BaseCamera

logger_mp = logging_mp.getLogger(__name__)


class UVCCamera(BaseCamera):
    def __init__(
        self,
        cam_topic,
        uid,
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
        import uvc

        self.uid = uid
        self.cap = None
        try:
            self.cap = uvc.Capture(self.uid)
        except Exception as e:
            self.cap = None
            raise RuntimeError(
                f"[UVCCamera] Failed to open camera {self._cam_topic}: {e}"
            )

        try:
            self.cap.frame_mode = self._choose_mode(
                self.cap,
                width=self._img_shape[1],
                height=self._img_shape[0],
                fps=self._fps,
            )
            logger_mp.info(str(self))
        except Exception as e:
            self.cap = None
            raise RuntimeError(
                f"[UVCCamera] Failed to set mode for {self._cam_topic}: {e}"
            )

    def __str__(self):
        return (
            f"[UVCCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS, MJPG.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def _choose_mode(self, cap, width=None, height=None, fps=None):
        for m in cap.available_modes:
            if (
                m.width == width
                and m.height == height
                and m.fps == fps
                and m.format_name == "MJPG"
            ):
                return m
        raise ValueError("[UVCCamera] No matching uvc mode found")

    def _update_frame(self):
        if self.cap is not None:
            frame = self.cap.get_frame_robust()  # get_frame(timeout=500)
            if frame is not None:
                self._webrtc_buffer.write(frame.bgr)

                if not self._ready.is_set():
                    self._ready.set()
            else:
                raise RuntimeError

    def release(self):
        # if usbhub is plugged out, calling stop_streaming and close may hang forever.
        # try:
        #     self.cap.stop_streaming()
        # except Exception:
        #     pass
        # try:
        #     self.cap.close()
        # except Exception:
        #     pass
        # self.cap = None
        logger_mp.info(f"[UVCCamera] Released {self._cam_topic}")
