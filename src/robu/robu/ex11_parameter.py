import rclpy
from rclpy.node import Node

class MinimalParameter(Node):
    def __init__(self):
        super().__init__('MinimalParameter')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('forward_speed_wf_slow', 0.05),
                ('forward_speed_wf_fast', 0.1),
                ('turning_speed_wf_slow', 0.1),
                ('turning_speed_wf_fast', 1.0),
                ('dist_thresh_wf', 0.3),
                ('dist_hysteresis_wf', 0.02),
            ])

        
        self.declare_parameter('my_parameter')

        #my_param = self.get_parameter('my_parameter')
        #print("my_parameter: ", my_param)

        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        forward_speed_wf_slow = self.get_parameter('forward_speed_wf_slow').value
        forward_speed_wf_fast = self.get_parameter('forward_speed_wf_fast').value
        turning_speed_wf_slow = self.get_parameter('turning_speed_wf_slow').value
        turning_speed_wf_fast = self.get_parameter('turning_speed_wf_fast').value
        dist_thresh_wf = self.get_parameter('dist_thresh_wf').value
        dist_hysteresis_wf = self.get_parameter('dist_hysteresis_wf').value

        #Ausgabe der Werte am Bildschirm
        #Syntax für print fund
        #('Formatstring' % (armugment1, argument2, argument3))

        self.get_logger().info('forward_speed_wf_slow: %5.2f forward_speed_wf_fast: %5.2f'
                               %(forward_speed_wf_slow, forward_speed_wf_fast))
        self.get_logger().info('turning_speed_wf_slow: %5.2f turning_speed_wf_fast: %5.2f'
                               %(turning_speed_wf_slow, turning_speed_wf_fast))
        self.get_logger().info('dist_thresh_wf: %5.2f dist_hysteresis_wf: %5.2f'
                               %(dist_thresh_wf, dist_hysteresis_wf))


def main():
    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)
