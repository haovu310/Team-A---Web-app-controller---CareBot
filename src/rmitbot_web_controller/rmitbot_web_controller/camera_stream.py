#!/usr/bin/env python3
"""
Camera Stream Node for RMITBot Web Controller
Subscribes to camera images and serves them as MJPEG stream over HTTP
Supports both CompressedImage (hardware) and Image (simulation)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import http.server
import socketserver
import threading
import io
import numpy as np

class CameraStreamHandler(http.server.BaseHTTPRequestHandler):
    """HTTP handler for serving MJPEG stream"""
    
    latest_frame = None
    frame_lock = threading.Lock()
    
    def do_GET(self):
        if self.path == '/camera/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            
            while True:
                try:
                    with self.frame_lock:
                        if self.latest_frame is not None:
                            frame_data = self.latest_frame
                        else:
                            continue
                    
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(frame_data)))
                    self.end_headers()
                    self.wfile.write(frame_data)
                    self.wfile.write(b"\r\n")
                    
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception as e:
                    break
        else:
            self.send_error(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress HTTP log messages"""
        return

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        
        # Declare parameters
        self.declare_parameter('stream_port', 8001)
        self.declare_parameter('camera_topic', 'camera/image_raw/compressed')
        self.declare_parameter('use_compressed', True)
        
        stream_port = self.get_parameter('stream_port').value
        camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        
        self.bridge = CvBridge()
        
        if self.use_compressed:
            # Subscribe to compressed camera images
            self.subscription = self.create_subscription(
                CompressedImage,
                camera_topic,
                self.compressed_callback,
                10
            )
            self.get_logger().info(f'Subscribing to CompressedImage: {camera_topic}')
        else:
            # Subscribe to raw camera images (e.g. from Gazebo)
            self.subscription = self.create_subscription(
                Image,
                camera_topic,
                self.raw_callback,
                10
            )
            self.get_logger().info(f'Subscribing to Raw Image: {camera_topic}')
            
        # Start HTTP server for streaming
        self.get_logger().info(f'Starting camera stream server on port {stream_port}')
        self.get_logger().info(f'Stream URL: http://localhost:{stream_port}/camera/stream')
        
        try:
            handler = CameraStreamHandler
            self.httpd = socketserver.TCPServer(("", stream_port), handler)
            
            # Run server in separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.get_logger().info('Camera stream server started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start camera stream server: {e}')
    
    def compressed_callback(self, msg):
        """Callback for compressed camera images"""
        with CameraStreamHandler.frame_lock:
            # compressed image data is already in JPEG format
            CameraStreamHandler.latest_frame = bytes(msg.data)
            
    def raw_callback(self, msg):
        """Callback for raw camera images (converts to JPEG)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Compress to JPEG
            _, jpeg_data = cv2.imencode('.jpg', cv_image)
            
            with CameraStreamHandler.frame_lock:
                CameraStreamHandler.latest_frame = jpeg_data.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')
    
    def stop(self):
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
            self.httpd.server_close()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
