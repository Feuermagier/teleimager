import threading

from teleimager.ring_buffer import TripleRingBuffer


class BaseCamera:
    def __init__(
        self,
        cam_topic,
        img_shape,
        fps,
        enable_zmq=True,
        zmq_port=55555,
        enable_webrtc=False,
        webrtc_port=66666,
        webrtc_codec=None,
    ):
        self._ready = threading.Event()
        self._cam_topic = cam_topic
        self._img_shape = img_shape  # (H, W)
        self._fps = fps
        self.enable_zmq = enable_zmq
        self._zmq_port = zmq_port

        self.enable_webrtc = enable_webrtc
        self._webrtc_port = webrtc_port
        self._webrtc_codec = webrtc_codec
        if self.enable_webrtc:
            self._webrtc_buffer = TripleRingBuffer()
        else:
            self._webrtc_buffer = None

    def __str__(self):
        raise NotImplementedError

    def __repr__(self):
        return self.__str__()

    def _update_frame(self):
        raise NotImplementedError

    def wait_until_ready(self, timeout=None):
        """Block until the camera is ready (first frame is available) or timeout occurs."""
        return self._ready.wait(timeout=timeout)

    def get_bgr_frame(self):
        return self._webrtc_buffer.read()

    def get_depth_frame(self):
        """Return a depth frame as bytes, or None if not supported.
        Before call this function, must first call get_frame() to update the latest depth data."""
        return None

    def get_zmq_port(self):
        """Return the zmq port number the camera is serving on."""
        return self._zmq_port

    def get_webrtc_port(self):
        """Return the webrtc port number the camera is serving on."""
        return self._webrtc_port

    def get_webrtc_codec(self):
        """Return the webrtc codec setting."""
        return self._webrtc_codec

    def get_fps(self):
        """Return the camera FPS setting."""
        return self._fps

    def release(self):
        """Release camera resources."""
        raise NotImplementedError
