import numpy as np
import time
import threading

# webrtc dependencies
import asyncio
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender
from aiortc.contrib.media import MediaRelay
from aiortc.codecs import h264
import av
import ssl
import cv2
import queue
import fractions
from typing import Dict, Optional, Tuple, Any
import logging_mp

logger_mp = logging_mp.getLogger(__name__)

INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC Stream</title>
    <style>
    body { 
        font-family: sans-serif; 
        background: #fff; 
        color: #000; 
        text-align: center; 
    }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; }
    video { width: 100%; max-width: 1280px; background: #000; margin-top: 10px; }
    
    /* Title link style */
    h1 a {
        text-decoration: none;
        color: #000;
    }
    h1 a:hover {
        color: #555;
    }
    </style>
</head>
<body>
    <h1>
        <a href="https://github.com/unitreerobotics/teleimager" target="_blank">
            XR Teleoperation WebRTC Camera Stream
        </a>
    </h1>

    <div style="margin-bottom: 20px;">
        <a href="https://www.unitree.com/" target="_blank">
            <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="10%">
        </a>
    </div>

    <button id="start" onclick="start()">Start</button>
    <button id="stop" style="display: none" onclick="stop()">Stop</button>
    
    <div id="media">
        <video id="video" autoplay playsinline muted></video>
        <audio id="audio" autoplay></audio>
    </div>
    
    <script src="client.js"></script>
</body>
</html>
"""

CLIENT_JS = """
var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        return response.json();
    }).then((answer) => {
        return pc.setRemoteDescription(answer);
    }).catch((e) => {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    // Removed STUN server check logic completely

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';
    if (pc) {
        pc.close();
        pc = null;
    }
}
"""


def jetson_software_encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
    if self.codec and (
        frame.width != self.codec.width or frame.height != self.codec.height
    ):
        self.codec = None

    if self.codec is None:
        if False:
            # software encoder, necessary for jetson
            self.codec = av.CodecContext.create("libx264", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(30, 1)
            self.codec.time_base = fractions.Fraction(1, 30)

            self.codec.options = {
                "preset": "ultrafast",
                "tune": "zerolatency",
                # "g": "12",
                # "g": "30",
                "crf": "23",  # higher == worse quality, lower == better quality
                "maxrate": "6M",
                "bufsize": "500k",  # small buffer to prevent frame buffering
                "intra-refresh": "1",  # avoid large keyframes
                "threads": "4",
                "sliced_threads": "1",  # prevent frame buffering from threading
                "no-mbtree": "1",  # Disable macroblock tree (lookahead)
                "rc-lookahead": "0",  # Ensure zero lookahead
            }
        else:
            # hardware encoder, should run on proper pc with nvidia gpu
            self.codec = av.CodecContext.create("h264_nvenc", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(30, 1)
            self.codec.time_base = fractions.Fraction(1, 30)

            self.codec.options = {
                "preset": "p1",  # p1 is "fastest/lowest latency" in NVENC
                "tune": "ull",  # Ultra-Low Latency
                "rc": "vbr",  # Variable Bitrate
                "cq": "22",  # Constant Quality (similar to CRF)
                "delay": "0",  # Force zero-frame delay
                "forced-idr": "1",  # Ensure I-frames are clean
                "g": "60",
                "bf": "0",
                "rc-lookahead": "0",
                "no-scenecut": "1",
            }
            # self.codec.options = {
            #     "preset": "p1",  # p1 is "fastest/lowest latency" in NVENC
            #     "tune": "ull",  # Ultra-Low Latency
            #     "rc": "cbr_ld_hq",  # constant low-delay high-quality bitrate
            #     # "cq": "22",  # Constant Quality (similar to CRF)
            #     "delay": "0",  # Force zero-frame delay
            #     "forced-idr": "1",  # Ensure I-frames are clean
            #     "bufsize": str(self.target_bitrate // 30),  # ~1 frame buffer
            #     "maxrate": str(self.target_bitrate),
            #     "bf": "0", # disable b-frames which increase latency
            #     "g": "30",      # 1 second GOP at 30fps to allow faster recovery on packet loss
            #     "keyint_min": "30",

            #     # specific nvenc settings for low latency
            #     "rc-lookahead": "0",     # disable lookahead
            #     "no-scenecut": "1",      # prevent sudden I-frame bursts
            #     "spatial_aq": "1",       # improves perceptual quality
            #     "temporal_aq": "0",      # disable (adds latency)
            #     "zerolatency": "1",      # enforce no reordering
            # }
            # self.codec.options = {
            #     # "profile": "main",
            #     "preset": "p1",  # p1 is "fastest/lowest latency" in NVENC
            #     "tune": "ull",  # Ultra-Low Latency
            #     "rc": "cbr_ld_hq",  # constant low-delay high-quality bitrate
            #     # "cq": "22",  # Constant Quality (similar to CRF)
            #     "delay": "0",  # Force zero-frame delay
            #     # "forced-idr": "1",  # Ensure I-frames are clean
            #     "bufsize": str(self.target_bitrate // 5),  # ~1 frame buffer
            #     "maxrate": str(self.target_bitrate),
            #     "bf": "0", # disable b-frames which increase latency
            #     "g": "30",      # 1 second GOP at 30fps to allow faster recovery on packet loss
            #     "keyint_min": "30",

            #     # specific nvenc settings for low latency
            #     "rc-lookahead": "0",     # disable lookahead
            #     "no-scenecut": "1",      # prevent sudden I-frame bursts
            #     "spatial_aq": "1",       # improves perceptual quality
            #     "aq-strength": "8",
            #     "temporal_aq": "0",      # disable (adds latency)
            # }

        self.frame_count = 0
        force_keyframe = True

    # if not force_keyframe and hasattr(self, "frame_count") and self.frame_count % 60 == 0:
    #     force_keyframe = True

    self.frame_count = self.frame_count + 1 if hasattr(self, "frame_count") else 1
    frame.pict_type = (
        av.video.frame.PictureType.I
        if force_keyframe
        else av.video.frame.PictureType.NONE
    )

    try:
        # t = time.perf_counter()
        packets = self.codec.encode(frame)
        # print("encoding time", (time.perf_counter() - t) * 1000)
    except Exception as e:
        print("encoding failed", e)

    total_size = 0
    for packet in packets:
        data = bytes(packet)
        total_size += len(data)
        if data:
            yield from self._split_bitstream(data)
    # print("outgoing kbytes", total_size / 1000)


h264.H264Encoder._encode_frame = jetson_software_encode_frame


class BGRArrayVideoStreamTrack(MediaStreamTrack):
    """MediaStreamTrack exposing BGR ndarrays as av.VideoFrame (latest-frame semantics)."""

    kind = "video"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._start_time = None
        self._pts = 0

    async def recv(self) -> av.VideoFrame:
        # This will suspend execution until a frame is available
        # preventing CPU busy-waiting
        frame = await self._queue.get()
        return frame

    def push_frame(
        self, bgr_numpy: np.ndarray, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        if bgr_numpy is None:
            return

        # 1. Convert and calculate PTS immediately
        # MediaRelay requires consistent PTS to function correctly
        try:
            yuv_frame = cv2.cvtColor(bgr_numpy, cv2.COLOR_BGR2YUV_I420)
            video_frame = av.VideoFrame.from_ndarray(yuv_frame, format="yuv420p")

            if self._start_time is None:
                self._start_time = time.time()
                self._pts = 0
            else:
                # 90000 is the standard RTP clock rate for video
                # This ensures smooth playback
                self._pts = int((time.time() - self._start_time) * 90000)

            video_frame.pts = self._pts
            video_frame.time_base = fractions.Fraction(1, 90000)

        except Exception as e:
            logger_mp.debug(f"Conversion failed: {e}")
            return

        # 2. Push to queue thread-safely
        target_loop = loop or asyncio.get_event_loop()
        if target_loop.is_closed():
            return

        def _put():
            try:
                # Drop old frame if queue is full (Low Latency strategy)
                if self._queue.full():
                    self._queue.get_nowait()
                self._queue.put_nowait(video_frame)
            except Exception:
                pass

        target_loop.call_soon_threadsafe(_put)


class WebRTC_PublisherThread(threading.Thread):
    """
    Runs aiohttp + aiortc in a separate THREAD (not Process).
    This enables shared memory and removes Pickling overhead.
    """

    def __init__(
        self,
        cert_path: str,
        key_path: str,
        port: int,
        host: str = "0.0.0.0",
        codec_pref: str = None,
    ):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self.cert_path = cert_path
        self.key_path = key_path
        self._codec_pref = codec_pref
        self._app = web.Application()
        self._runner: Optional[web.AppRunner] = None
        self._pcs = set()
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._frame_available_event = asyncio.Event()

        self._bgr_track: Optional[BGRArrayVideoStreamTrack] = None
        self._relay: Optional[MediaRelay] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # register routes
        self._app.router.add_get("/", self._index)
        self._app.router.add_get("/client.js", self._javascript)
        self._app.router.add_post("/offer", self._offer)

        self._app.router.add_options("/", self._options)
        self._app.router.add_options("/client.js", self._options)
        self._app.router.add_options("/offer", self._options)

    async def _index(self, request: web.Request) -> web.Response:
        return web.Response(content_type="text/html", text=INDEX_HTML)

    async def _javascript(self, request: web.Request) -> web.Response:
        return web.Response(content_type="application/javascript", text=CLIENT_JS)

    async def _options(self, request):
        return web.Response(
            status=200,
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "POST, GET, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            },
        )

    async def _offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        # CORE LOGIC: Use MediaRelay to subscribe
        # This ensures encoding happens only once globally
        if self._bgr_track and self._relay:
            try:
                relayed_track = self._relay.subscribe(self._bgr_track)
                transceiver = pc.addTransceiver(relayed_track, direction="sendonly")
                capabilities = RTCRtpSender.getCapabilities("video")
                pref = (self._codec_pref or "h264").lower()

                if pref == "h264":
                    h264_codecs = [
                        c for c in capabilities.codecs if c.mimeType == "video/H264"
                    ]
                    if h264_codecs:
                        transceiver.setCodecPreferences(h264_codecs)
                        logger_mp.info(f"[WebRTC] Preferred H264 for port:{self._port}")
                    else:
                        logger_mp.warning(
                            f"[WebRTC] H264 preferred but not found, using auto-negotiation for port:{self._port}"
                        )

                elif pref == "vp8":
                    vp8_codecs = [
                        c for c in capabilities.codecs if c.mimeType == "video/VP8"
                    ]
                    if vp8_codecs:
                        transceiver.setCodecPreferences(vp8_codecs)
                        logger_mp.info(f"[WebRTC] Preferred VP8 for port:{self._port}")
                    else:
                        logger_mp.warning(
                            f"[WebRTC] VP8 preferred but not found, using auto-negotiation for port:{self._port}"
                        )

                else:
                    h264_codecs = [
                        c for c in capabilities.codecs if c.mimeType == "video/H264"
                    ]
                    if h264_codecs:
                        transceiver.setCodecPreferences(h264_codecs)
                        logger_mp.info(
                            f"[WebRTC] Preferred codec '{pref}' not found, falling back to H264 for port:{self._port}"
                        )
                    else:
                        logger_mp.warning(
                            f"[WebRTC] Preferred codec '{pref}' not found, using auto-negotiation for port:{self._port}"
                        )

            except Exception as e:
                logger_mp.error(f"Relay subscription failed: {e}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState in ["failed", "closed"]:
                await self._cleanup_pc(pc)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            },
        )

    async def _cleanup_pc(self, pc):
        self._pcs.discard(pc)
        try:
            await pc.close()
        except:
            pass

    def wait_for_start(self, timeout=1.0):
        return self._start_event.wait(timeout=timeout)

    def run(self):
        # Create a new Event Loop for this thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def _main():
            self._runner = web.AppRunner(self._app)
            await self._runner.setup()

            # Init Track and Relay inside the loop
            self._bgr_track = BGRArrayVideoStreamTrack()
            self._relay = MediaRelay()

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(self.cert_path, self.key_path)
            site = web.TCPSite(
                self._runner, self._host, self._port, ssl_context=ssl_context
            )
            await site.start()
            self._start_event.set()

            # Frame Pushing Loop
            while not self._stop_event.is_set():
                await self._frame_available_event.wait()
                self._frame_available_event.clear()
                frame, t = self._latest_frame
                # print("webrtc", (time.perf_counter() - t) * 1000)
                self._bgr_track.push_frame(frame, loop=self._loop)

        try:
            self._loop.run_until_complete(_main())
        except Exception as e:
            logger_mp.error(f"WebRTC Thread Error: {e}")
        finally:
            if self._loop:
                self._loop.close()

    def send(self, data):
        """Send data to the processing thread."""
        self._latest_frame = data
        self._loop.call_soon_threadsafe(self._frame_available_event.set)

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)


class WebRTC_PublisherManager:
    """Manages WebRTC_PublisherThreads."""

    def __init__(self, cert_path: str, key_path: str):
        self.cert_path = cert_path
        self.key_path = key_path

        self._publisher_threads: Dict[Tuple[str, int], WebRTC_PublisherThread] = {}
        self._lock = threading.Lock()

    def _create_publisher(self, port: int, host: str, codec_pref: str):
        t = WebRTC_PublisherThread(
            self.cert_path, self.key_path, port, host, codec_pref
        )
        t.start()
        if not t.wait_for_start(timeout=10.0):  # Increase timeout to 10 seconds
            raise ConnectionError("Publisher failed to start (Timeout)")
        return t

    def _get_publisher(self, port, host, codec_pref):
        key = (host, port)
        with self._lock:
            if key not in self._publisher_threads:
                self._publisher_threads[key] = self._create_publisher(
                    port, host, codec_pref
                )
            return self._publisher_threads[key]

    def publish(
        self, data: Any, port: int, host: str = "0.0.0.0", codec_pref: str = None
    ) -> None:
        try:
            pub = self._get_publisher(port, host, codec_pref)
            pub.send(data)
        except Exception as e:
            logger_mp.error(f"Unexpected error in publish: {e}")
            pass

    def close(self) -> None:
        with self._lock:
            for key, pub in list(self._publisher_threads.items()):
                try:
                    pub.stop()
                except Exception:
                    pass
            self._publisher_threads.clear()
