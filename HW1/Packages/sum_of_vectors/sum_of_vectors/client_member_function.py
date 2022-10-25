from addvectors.srv import AddTwoVectors                      # CHANGE
import sys
# import string
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoVectors, 'add_two_vectors')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoVectors.Request()                                   # CHANGE

    def send_request(self):
        str1 = sys.argv[1]
        str2 = sys.argv[2]
        self.req.a = [int(x) for x in (str1.split(','))]
        self.req.b = [int(x) for x in (str2.split(','))]
                                               # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_vectors: for %s + %s = %s' %                                # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, response.sum))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()