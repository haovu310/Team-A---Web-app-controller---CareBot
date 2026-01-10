#!/usr/bin/env python3
"""
Camera Stream Node for RMITBot Web Controller
Subscribes to camera images and serves them as MJPEG stream over HTTP
OPTIMIZED FOR RASPBERRY PI - Reduced CPU usage and better connection handling
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import http.server
import socketserver
import threading
import io
import time

class CameraStreamHandler(http.server.BaseHTTPRequestHandler):
    """HTTP handler for serving MJPEG stream"""

    latest_frame = None
    frame_lock = threading.Lock()
    frame_timestamp = 0
    max_fps = 15  # Limit FPS to reduce CPU load
    client_timeout = 10.0  # Timeout for inactive clients
    
    def do_GET(self):
        if self.path == '/camera/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Connection', 'close')
            self.end_headers()

            last_send_time = 0
            last_activity = time.time()
            min_frame_interval = 1.0 / self.max_fps

            while True:
                try:
                    current_time = time.time()

                    # Client timeout check
                    if current_time - last_activity > self.client_timeout:
                        break

                    # Frame rate limiting - prevent sending frames too fast
                    if current_time - last_send_time < min_frame_interval:
                        time.sleep(0.01)  # Small sleep to prevent busy-wait
                        continue

                    with self.frame_lock:
                        if self.latest_frame is not None:
                            frame_data = self.latest_frame
                            frame_time = self.frame_timestamp
                        else:
                            time.sleep(0.02)
                            continue

                    # Skip duplicate frames
                    if frame_time <= last_send_time:
                        time.sleep(0.01)
                        continue

                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame_data)}\r\n".encode())
                    self.wfile.write(b"\r\n")
                    self.wfile.write(frame_data)
                    self.wfile.write(b"\r\n")

                    last_send_time = current_time
                    last_activity = current_time

                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
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
        self.declare_parameter('max_fps', 15)  # RPi-optimized FPS limit

        stream_port = self.get_parameter('stream_port').value
        camera_topic = self.get_parameter('camera_topic').value
        max_fps = self.get_parameter('max_fps').value

        # Set FPS limit in handler
        CameraStreamHandler.max_fps = max_fps

        # Frame rate limiting at subscription level
        self.last_process_time = 0
        self.min_process_interval = 1.0 / max_fps

        # Subscribe to compressed camera images with SMALL queue
        self.subscription = self.create_subscription(
            CompressedImage,
            camera_topic,
            self.image_callback,
            1  # Reduced queue size prevents lag buildup
        )
        
        # Start HTTP server for streaming
        self.get_logger().info(f'Starting camera stream server on port {stream_port}')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info(f'Max FPS: {max_fps} (optimized for Raspberry Pi)')
        self.get_logger().info(f'Stream URL: http://localhost:{stream_port}/camera/stream')

        try:
            # Use ThreadingMixIn for better concurrent client handling
            class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
                daemon_threads = True
                allow_reuse_address = True
                request_queue_size = 3  # Limit queue to prevent overload

            handler = CameraStreamHandler
            self.httpd = ThreadedTCPServer(("", stream_port), handler)
            
            # Run server in separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.get_logger().info('Camera stream server started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start camera stream server: {e}')
    
    def image_callback(self, msg):
        """Callback for camera images with frame rate limiting"""
        # Frame rate limiting - skip frames if processing too fast
        current_time = time.time()
        if current_time - self.last_process_time < self.min_process_interval:
            return  # Skip this frame

        with CameraStreamHandler.frame_lock:
            # compressed image data is already in JPEG format
            CameraStreamHandler.latest_frame = bytes(msg.data)
            CameraStreamHandler.frame_timestamp = current_time

        self.last_process_time = current_time
    
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
