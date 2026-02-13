import cv2
import logging_mp

from teleimager.cameras.base_camera import BaseCamera

logger_mp = logging_mp.get_logger(__name__)


class IsaacSimCamera(BaseCamera):
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
        image_source="head",
        binocular=False,
    ):
        """
        IsaacSim camera that reads from shared memory.

        Args:
            cam_topic: camera topic name
            img_shape: image shape [height, width]
            fps: frames per second
            enable_zmq: enable ZMQ publishing
            zmq_port: ZMQ port
            enable_webrtc: enable WebRTC publishing
            webrtc_port: WebRTC port
            webrtc_codec: WebRTC codec preference
            image_source: which image to read from shared memory ("head", "left", "right")
            binocular: if True and image_source=="head", concatenate left+right for binocular vision
        """
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
        from tools.shared_memory_utils import (
            MultiImageReader,
        )  # https://github.com/unitreerobotics/unitree_sim_isaaclab/tree/main/tools

        self.multi_image_reader = MultiImageReader()
        self._image_source = image_source  # "head", "left", or "right"
        self._binocular = binocular
        # For IsaacSim cameras, set ready immediately since the camera object is initialized
        # and will wait for shared memory data in _update_frame
        self._ready.set()
        logger_mp.info(str(self))

    def __str__(self):
        mode = "binocular" if self._binocular else "monocular"
        return (
            f"[IsaacSimCamera: {self._cam_topic}] initialized with "
            f"{self._img_shape[0]}x{self._img_shape[1]} @ {self._fps} FPS, source='{self._image_source}', mode='{mode}'.\n"
            f"ZMQ: {'enabled, zmq port=' + str(self._zmq_port) if self._enable_zmq else 'disabled'}; "
            f"WebRTC: {'enabled, webrtc port=' + str(self._webrtc_port) if self._enable_webrtc else 'disabled'}"
        )

    def _update_frame(self):
        # Get the image data based on source and binocular settings
        frame_data = None
        if self._binocular:
            # For binocular cameras: concatenate left + right images
            left_img = self.multi_image_reader.read_single_image("left")
            right_img = self.multi_image_reader.read_single_image("right")
            logger_mp.debug(
                f"[IsaacSimCamera] {self._cam_topic} - left: {left_img is not None}, right: {right_img is not None}"
            )

            if left_img is not None and right_img is not None:
                frame_data = cv2.hconcat([left_img, right_img])
                logger_mp.debug(
                    f"[IsaacSimCamera] {self._cam_topic} - concatenated binocular frame: {frame_data.shape}"
                )
        else:
            # For monocular cameras: use the specified source directly
            frame_data = self.multi_image_reader.read_single_image(self._image_source)
            if frame_data is None:
                logger_mp.debug(
                    f"[IsaacSimCamera] {self._cam_topic} - no data for source '{self._image_source}'"
                )

        # Publish the frame data only if we have valid data
        if frame_data is not None:
            self._webrtc_buffer.write(frame_data)
            if not self._ready.is_set():
                self._ready.set()
        else:
            logger_mp.debug(
                f"[IsaacSimCamera] No data available for {self._cam_topic}, frame_data is None"
            )
        # If no data is available, just return silently and wait for next frame

    def release(self):
        if hasattr(self, "multi_image_reader") and self.multi_image_reader is not None:
            self.multi_image_reader.close()
        self.multi_image_reader = None
        logger_mp.info(f"[IsaacSimCamera] Released {self._cam_topic}")
