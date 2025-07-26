#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import ProcessCoffee

class PhotoServiceClient(Node):
    def __init__(self):
        super().__init__('photo_service_client')
        self.cli = self.create_client(ProcessCoffee, 'process_coffee')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
    def send_request(self, status=2):
        request = ProcessCoffee.Request()
        request.status = status
        
        future = self.cli.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    
    client = PhotoServiceClient()
    
    # Send request with status 2 to take photo
    future = client.send_request(status=2)
    
    rclpy.spin_until_future_complete(client, future)
    
    if future.result() is not None:
        response = future.result()
        if response.photo_taken:
            client.get_logger().info('Photo captured successfully!')
        else:
            client.get_logger().error('Failed to capture photo')
    else:
        client.get_logger().error('Service call failed')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
