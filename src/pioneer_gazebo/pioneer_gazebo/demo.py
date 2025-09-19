from framework_core.decorators import node, publisher, timer
from framework_core.helpers import spin_node
from geometry_msgs.msg import Twist
import sys, termios, tty, select, os

def has_tty():
    return os.isatty(sys.stdin.fileno())

# --- non-blocking key reader (no echo) ---
def get_key_nonblocking():
    if not has_tty():
        return None   # no keyboard available
    
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # raw mode, no echo
        tty.setraw(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


@node("teleop")
class Teleop:
    """
    Teleop node that publishes velocity commands to /cmd_vel
    using WASD keys. Space stops, 'q' quits.
    """

    def __init__(self):
        self.twist = Twist()
        self.logger.info("Teleop started. Use WASD to move, space to stop, 'q' to quit.")

    @timer(period_sec=0.1)  # 10 Hz loop
    @publisher(topic="cmd_vel", msg_type=Twist)
    def publish_cmd(self):
        key = get_key_nonblocking()

        # reset to zero each cycle unless overridden
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if key == 'w':
            self.twist.linear.x = 1.5
        elif key == 's':
            self.twist.linear.x = -1.5
        elif key == 'a':
            self.twist.angular.z = 1.5
        elif key == 'd':
            self.twist.angular.z = -1.5
        elif key == 'q':
            self.logger.info("Quitting teleop…")
            raise SystemExit
        # space already handled by reset-to-zero above

        return self.twist


def main(args=None):
    spin_node(Teleop, args)


if __name__ == "__main__":
    main()
