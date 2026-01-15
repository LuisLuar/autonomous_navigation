#!/usr/bin/env python3
"""
webrtc_streamer.py - Nodo ROS2 que sirve de puente entre ROS2 y un servidor webrtc
Flask + aiortc server: toma frames desde 3 tópicos ROS2 diferentes
y los envía por WebRTC en 3 endpoints diferentes.
"""
import asyncio
import threading
import logging
import traceback
from concurrent.futures import ThreadPoolExecutor
from enum import Enum

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from av import VideoFrame

from flask import Flask, request, jsonify
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
handler.setFormatter(formatter)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc_streamer_multi")
logger.addHandler(handler)
logger.setLevel(logging.INFO)

# Silenciar otros loggers
for lib in ['aioice', 'werkzeug', 'websockets', 'av']:
    logging.getLogger(lib).setLevel(logging.WARNING)

class StreamType(Enum):
    CAMERA = "camera"
    DETECTION = "detection" 
    SEGMENTATION = "segmentation"

# ---------------- ROS2 multi-stream subscriber ----------------
class RosMultiStream(Node):
    def __init__(self):
        super().__init__("webrtc_stream")
        self.bridge = CvBridge()
        
        # Almacenar frames más recientes para cada stream
        self.latest_frames = {
            StreamType.CAMERA: None,
            StreamType.DETECTION: None,
            StreamType.SEGMENTATION: None
        }
        
        self.frame_counts = {
            StreamType.CAMERA: 0,
            StreamType.DETECTION: 0,
            StreamType.SEGMENTATION: 0
        }

        # Suscriptor para cámara RGB original
        self.subscription_camera = self.create_subscription(Image,"/camera/rgb/image_raw",self.camera_listener,10)

        # Suscriptor para detecciones
        self.subscription_detection = self.create_subscription(Image,"/detection/annotated_image",self.detection_listener,10) #/segmentation/droidcam/overlay /detection/annotated_image 

        # Suscriptor para segmentación
        self.subscription_segmentation = self.create_subscription(Image,"/segmentation/overlay",self.segmentation_listener,10)

        #logger.info("ROS2 multi-stream subscribers created for 3 topics")

    def camera_listener(self, msg: Image):
        try:
            self.latest_frames[StreamType.CAMERA] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_counts[StreamType.CAMERA] += 1
            #if self.frame_counts[StreamType.CAMERA] % 120 == 0:
                #logger.info(f"ROS2 camera frames: {self.frame_counts[StreamType.CAMERA]}")
        except Exception as e:
            #logger.exception("Error converting camera image: %s", e)
            pass

    def detection_listener(self, msg: Image):
        try:
            self.latest_frames[StreamType.DETECTION] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_counts[StreamType.DETECTION] += 1
            #if self.frame_counts[StreamType.DETECTION] % 120 == 0:
                #logger.info(f"ROS2 detection frames: {self.frame_counts[StreamType.DETECTION]}")
        except Exception as e:
            #logger.exception("Error converting detection image: %s", e)
            pass

    def segmentation_listener(self, msg: Image):
        try:
            self.latest_frames[StreamType.SEGMENTATION] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_counts[StreamType.SEGMENTATION] += 1
            #if self.frame_counts[StreamType.SEGMENTATION] % 120 == 0:
                #logger.info(f"ROS2 segmentation frames: {self.frame_counts[StreamType.SEGMENTATION]}")
        except Exception as e:
            #logger.exception("Error converting segmentation image: %s", e)
            pass

    def get_latest_frame(self, stream_type: StreamType):
        return self.latest_frames.get(stream_type)
    
    def destroy_node(self):
        super().destroy_node()

# ---------------- WebRTC video track para múltiples streams ----------------
class RosVideoTrack(VideoStreamTrack):
    def __init__(self, ros_node: RosMultiStream, stream_type: StreamType):
        super().__init__()
        self.ros_node = ros_node
        self.stream_type = stream_type
        self.sent_frames = 0
        #logger.info(f"RosVideoTrack initialized for: {stream_type.value}")

    async def recv(self):
        # Target ~30 FPS
        await asyncio.sleep(1 / 30.0)

        img = self.ros_node.get_latest_frame(self.stream_type)
        if img is None:
            # Frame placeholder
            img = np.zeros((480, 640, 3), dtype=np.uint8)

            # Mensajes según tipo de stream
            messages = {
                "CAMERA": "NO EXISTE FRAME DE CAMARA",
                "DETECTION": "NO EXISTE FRAME DE DETECCION",
                "SEGMENTATION": "NO EXISTE FRAME DE SEGMENTACION"
            }
            
            stream_upper = self.stream_type.value.upper()
            message = messages.get(stream_upper, f"NO {stream_upper} FRAME")
            
            # Texto centrado básico
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, message, 
                    (img.shape[1]//4, img.shape[0]//2),
                    font, 0.8, (255, 255, 255), 2)

        else:
            if img.dtype != np.uint8:
                img = img.astype(np.uint8)

        frame = VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts, frame.time_base = await self.next_timestamp()
        
        self.sent_frames += 1
        #if self.sent_frames % 300 == 0:
            #logger.info(f"Frames sent via WebRTC ({self.stream_type.value}): {self.sent_frames}")
        return frame

# ---------------- WebRTC event loop ----------------
webrtc_loop = None
webrtc_executor = ThreadPoolExecutor(max_workers=2)

def setup_webrtc_loop():
    global webrtc_loop
    webrtc_loop = asyncio.new_event_loop()
    def run_loop():
        asyncio.set_event_loop(webrtc_loop)
        webrtc_loop.run_forever()
    t = threading.Thread(target=run_loop, daemon=True)
    t.start()
    #logger.info("WebRTC event loop started in background thread")

async def create_peer_connection_async(offer: RTCSessionDescription, track: RosVideoTrack):
    """
    Crea conexión peer WebRTC para un stream específico
    """
    ice_servers = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
    pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ice_servers))
    #logger.info(f"Created RTCPeerConnection for {track.stream_type.value}")

    @pc.on("iceconnectionstatechange")
    async def on_ice_state_change():
        #logger.info(f"ICE state ({track.stream_type.value}): {pc.iceConnectionState}")
        if pc.iceConnectionState == "failed":
            await pc.close()

    # Configurar transceiver para enviar video
    pc.addTransceiver("video", direction="sendonly")
    pc.addTrack(track)

    # Establecer offer remota y crear answer
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    #logger.info(f"Created answer for {track.stream_type.value}")
    return pc, pc.localDescription

# ---------------- Flask app (signaling) con múltiples endpoints ----------------
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

pcs = set()
ros_node = None

@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "healthy",
        "ros_connected": ros_node is not None,
        "webrtc_loop_running": webrtc_loop is not None,
        "streams_available": ["camera", "detection", "segmentation"]
    })

# Endpoint para stream de cámara original
@app.route("/offer/camera", methods=["POST", "OPTIONS"])
def offer_camera():
    return handle_offer(StreamType.CAMERA)

# Endpoint para stream de detección
@app.route("/offer/detection", methods=["POST", "OPTIONS"])
def offer_detection():
    return handle_offer(StreamType.DETECTION)

# Endpoint para stream de segmentación  
@app.route("/offer/segmentation", methods=["POST", "OPTIONS"])
def offer_segmentation():
    return handle_offer(StreamType.SEGMENTATION)

def handle_offer(stream_type: StreamType):
    if request.method == "OPTIONS":
        return ('', 200)

    try:
        params = request.get_json()
        if not params:
            return jsonify({"error": "No JSON body"}), 400
        if "sdp" not in params or "type" not in params:
            return jsonify({"error": "Missing sdp/type"}), 400

        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Crear track para el stream específico
        track = RosVideoTrack(ros_node, stream_type)

        fut = asyncio.run_coroutine_threadsafe(
            create_peer_connection_async(offer, track),
            webrtc_loop
        )
        pc, local_desc = fut.result(timeout=30)
        pcs.add(pc)
        #logger.info(f"Answered offer for {stream_type.value}")
        return jsonify({"sdp": local_desc.sdp, "type": local_desc.type})
    except Exception as e:
        #logger.exception("Error in /offer/%s: %s", stream_type.value, e)
        return jsonify({"error": str(e)}), 500

# ---------------- main ----------------
def start_flask():
    #logger.info("Starting Flask on 0.0.0.0:8081")
    app.run(host="0.0.0.0", port=8081, debug=False, use_reloader=False)

def main():
    global ros_node
    #logger.info("Initializing rclpy and ROS2 multi-stream node")
    rclpy.init()
    ros_node = RosMultiStream()

    setup_webrtc_loop()

    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()

    #logger.info("Multi-stream server running on http://0.0.0.0:8081")
    #logger.info("Endpoints:")
    #logger.info("  - Health: http://0.0.0.0:8081/health")
    #logger.info("  - Camera: http://0.0.0.0:8081/offer/camera") 
    #logger.info("  - Detection: http://0.0.0.0:8081/offer/detection")
    #logger.info("  - Segmentation: http://0.0.0.0:8081/offer/segmentation")
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        #logger.info("Interrupted")
        pass
    finally:
        #logger.info("Cleaning up...")
        try:
            for pc in list(pcs):
                asyncio.run_coroutine_threadsafe(pc.close(), webrtc_loop).result(timeout=5)
        except Exception:
            pass
        if webrtc_loop:
            webrtc_loop.call_soon_threadsafe(webrtc_loop.stop)
        webrtc_executor.shutdown(wait=False)

        try:
            ros_node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == "__main__":
    main()