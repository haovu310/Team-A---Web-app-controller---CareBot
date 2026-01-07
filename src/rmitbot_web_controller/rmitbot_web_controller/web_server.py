import os
import sys
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import http.server
import socketserver
import threading
from pathlib import Path
from urllib.parse import urlparse, parse_qs

PORT = 8000

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP handler to add API endpoints"""

    def do_GET(self):
        # Parse the URL
        parsed_path = urlparse(self.path)

        # Handle /list_maps endpoint
        if parsed_path.path == '/list_maps':
            self.handle_list_maps()
        else:
            # Default behavior for static files
            super().do_GET()

    def handle_list_maps(self):
        """List all saved maps from ~/.ros directory"""
        try:
            # Get the ~/.ros directory
            ros_dir = Path.home() / '.ros'

            if not ros_dir.exists():
                maps = []
            else:
                # Find all .posegraph files (slam_toolbox saves maps as .posegraph and .data)
                posegraph_files = list(ros_dir.glob('*.posegraph'))
                # Extract just the base names (without extension)
                maps = [f.stem for f in posegraph_files]

            # Return JSON response
            response = {
                'maps': sorted(maps),
                'count': len(maps)
            }

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())

        except Exception as e:
            # Return error response
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            error_response = {
                'error': str(e),
                'maps': []
            }
            self.wfile.write(json.dumps(error_response).encode())

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.declare_parameter('port', 8000)
        self.port = self.get_parameter('port').value

        # Determine the path to the content directory
        try:
            package_share_directory = get_package_share_directory('rmitbot_web_controller')
            web_dir = os.path.join(package_share_directory, 'content')
            self.get_logger().info(f'Serving content from: {web_dir}')

            # Change directory to serve files
            os.chdir(web_dir)

            handler = CustomHTTPRequestHandler
            self.httpd = socketserver.TCPServer(("", self.port), handler)

            self.get_logger().info(f'Web Server running at port {self.port}')

            # Run server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

    def stop(self):
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
            self.httpd.server_close()

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
