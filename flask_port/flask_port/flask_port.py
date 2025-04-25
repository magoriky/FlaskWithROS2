# my_flask_app/app.py
from flask import Flask
from flask import jsonify

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

app = Flask(__name__)

class FlaskROS2Node(Node):
    def __init__(self):
        super().__init__('flask_ros2_node')


        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.client_trigger = self.create_client(Trigger, "trigger_me")
        while not self.client_trigger.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('trigger_me not available, waiting again...')

    def publish_message(self, message):
         msg = String()
         msg.data = message
         self.publisher_.publish(msg)
         self.get_logger().info(f'Publishing message: {message}')
    
    # def call_my_service_sync(self):
    #     req = Trigger.Request()
    #     response = self.client_trigger.call(req)
    #     return {
    #             'success': response.success,
    #             'message': response.message
    #             }
    def call_my_service_sync(self):
        req = Trigger.Request()
        future = self.client_trigger.call_async(req)
    
        rclpy.spin_until_future_complete(self, future)
    
        if future.result() is not None:
            return {
                'success': future.result().success,
                'message': future.result().message
            }
        else:
            return {
                'success': False,
                'message': 'Service call failed'
            }



# Initialize ROS 2 node outside the routes
rclpy.init()
flask_node = FlaskROS2Node()

@app.route('/send_message/<message>')
def send_message(message):
    flask_node.publish_message(message)
    return f'Sent message: {message}'

@app.route('/trigger', methods=['POST'])
def call_ros_service():
    response = flask_node.call_my_service_sync()
    return jsonify(response)

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)