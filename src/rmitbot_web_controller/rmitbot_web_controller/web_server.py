import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import http.server
import socketserver
import threading

PORT = 8000

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
            
            handler = http.server.SimpleHTTPRequestHandler
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
