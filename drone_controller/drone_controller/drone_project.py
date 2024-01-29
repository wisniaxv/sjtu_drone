import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Subskrybent aktualnej pozycji
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            1)

        self.gt_pose = None

        # Wydawca komendy kontroli
        self.command_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Callback do wykonywania komend kontroli
        timer_period = 0.1  # sekundy
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Punkt startowy (2 metry nad ziemią)
        self.starting_point = Point(x=0.0, y=0.0, z=2.0)

        # Czas rozpoczęcia ruchu
        self.start_time = time.time()

        # Aktualny etap lotu
        self.flight_stage = 0

    def pose_callback(self, data):
        self.gt_pose = data

    def timer_callback(self):
        if self.gt_pose is not None:
            # Sprawdź, czy minęło co najmniej 2 sekundy od rozpoczęcia ruchu
            elapsed_time = time.time() - self.start_time

            if elapsed_time >= 8.0:
                # Zmień etap lotu
                self.flight_stage += 1
                # Zresetuj czas rozpoczęcia dla nowego etapu
                self.start_time = time.time()

            # Wykonaj ruch zgodnie z aktualnym etapem
            self.execute_flight_stage()

    def execute_flight_stage(self):
        cmd = Twist()

        if self.flight_stage == 0:
            # Podnieś się o 1 metry (do góry)
            cmd.linear.z = 1.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 1:
            # Lecimy 2 metry w lewo
            cmd.linear.z = 3.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 2:
            # Lecimy 2 metry w dół
            cmd.linear.z = 3.0
            cmd.linear.x = 2.0
        elif self.flight_stage == 3:
            # Lecimy 2 metry w prawo
            cmd.linear.z = 1.0
            cmd.linear.x = 2.0
        elif self.flight_stage == 4:
            #wracamy
            cmd.linear.z = 1.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 5:
            #wracamy na ziemie
            cmd.linear.z = 0.0
            cmd.linear.x = 0.0

        # Wyślij komendę
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = DroneController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Przerwano przez użytkownika. Zamykanie.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






