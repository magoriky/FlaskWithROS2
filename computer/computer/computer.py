import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
class HelloServiceNode(Node):
    def __init__(self):
        super().__init__('hello_service_node')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.i = 1
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Service
        self.srv = self.create_service(Trigger, 'trigger_me', self.trigger_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello world {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1

    def trigger_callback(self, request, response):
        self.get_logger().info("Service was triggered!")
        for i in range(10):
            self.get_logger().info(f"Counter is {i}")
            time.sleep(1)
        response.success = True
        response.message = "you triggered me"
        return response

def main():
    rclpy.init()
    servoMotor = HelloServiceNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(servoMotor, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        servoMotor.destroy_node()
    except Exception as e:
        servoMotor.destroy_node()
        rclpy.shutdown()
    finally:
        servoMotor.destroy_node()
        rclpy.shutdown()
       

if __name__ == '__main__':
    main()    