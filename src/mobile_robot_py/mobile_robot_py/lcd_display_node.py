"""
ROS 2 node: LCDDisplayNode
---------------------------------
Displays selected fields from sensor_msgs/JointState on a simple serial LCD.

Behavior in short:
- Subscribes to the /joint_states topic.
- Formats up to `max_lines` lines (default 2), each up to MAX_CHARS_PER_LINE characters.
- Periodically (every `update_period` seconds) sends the formatted lines over a serial
    connection to an attached LCD microcontroller, one line per newline, ending with a final
    newline (so the payload has `max_lines + 1` newlines).
- Tries to (re)open the serial port on startup and whenever a write fails.

Parameters:
- serial_port (string): Serial device path (default: /dev/ttyACM0)
- baudrate (int): UART baud rate (default: 115200)
- max_lines (int): Number of LCD lines to output (default: 2)
- update_period (double): Update interval seconds (default: 0.5)

Notes:
- The node is tolerant of the serial device not being present; it will keep retrying.
- Access to the latest JointState is protected by a threading.Lock because the timer and
    subscription callback can run on different threads.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import threading
import time
import os

MAX_CHARS_PER_LINE = 16

class LCDDisplayNode(Node):
    def __init__(self):
        super().__init__('lcd_display_node')
    # Declare user-configurable parameters with defaults.
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_lines', 2)
        self.declare_parameter('update_period', 0.5)

    # Read parameter values once at startup.
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.max_lines = self.get_parameter('max_lines').get_parameter_value().integer_value
        self.update_period = self.get_parameter('update_period').get_parameter_value().double_value

        self.get_logger().info(f'LCD node serial={self.serial_port} baud={self.baudrate} update_period={self.update_period}')

    # Try to open serial (non-fatal if it fails; we'll retry later).
        self.ser = None
        self.try_open_serial()

    # Shared state: last received JointState message. Protect with a lock because
    # the timer and subscriber callback may run concurrently.
        self.joint_state = None
        self.lock = threading.Lock()

    # Subscribe to joint states and set up periodic updates to the LCD.
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_timer(self.update_period, self.timer_cb)

    def try_open_serial(self):
        """Attempt to open the serial port if not already open.

        This method is safe to call on every timer tick; it will no-op if already open.
        """
        if self.ser is not None:
            return
        try:
            if os.path.exists(self.serial_port):
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
                self.get_logger().info(f'Opened serial port {self.serial_port}')
            else:
                self.get_logger().warn(f'Serial port {self.serial_port} does not exist yet')
        except Exception as e:
            self.get_logger().warn(f'Could not open serial {self.serial_port}: {e}')
            self.ser = None

    def joint_cb(self, msg: JointState):
        """Subscriber callback for /joint_states.

        Stores the latest JointState so the timer can render it to the LCD.
        """
        with self.lock:
            self.joint_state = msg

    def format_lines(self, js: JointState):
        """Convert the latest JointState into LCD lines.

        Returns a list of length `max_lines`, each string trimmed to MAX_CHARS_PER_LINE.
        Format per line: "<name>:<pos>" with position at 2 decimal places. Underscores
        in names are removed to save space. If no data yet, first line shows "waiting...".
        """
        # Return list of strings (len == max_lines), each <= MAX_CHARS_PER_LINE.
        if js is None or len(js.name) == 0:
            return ['waiting...', '']
        lines = []
        for i in range(min(self.max_lines, len(js.name))):
            name = js.name[i]
            pos = js.position[i] if i < len(js.position) else 0.0
            # format: name:xx.xx -> try to keep short
            s = f'{name}:{pos:.2f}'
            # replace long underscores and make it compact
            s = s.replace('_', '')
            if len(s) > MAX_CHARS_PER_LINE:
                s = s[:MAX_CHARS_PER_LINE]
            lines.append(s)
        while len(lines) < self.max_lines:
            lines.append('')
        return lines

    def send_serial(self, lines):
        """Send the given lines to the LCD over serial.

        Concatenates lines with '\n' and appends a trailing '\n'. If a write fails, the
        serial handle is closed so a subsequent tick will attempt to reopen it.
        """
        if not self.ser:
            return
        try:
            payload = '\n'.join(lines) + '\n'
            self.ser.write(payload.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Failed to write serial: {e}')
            # close so next timer tick will attempt reopen
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def timer_cb(self):
        """Periodic task that (re)opens serial if needed, formats, and sends data."""
        # try reopen if serial not ready
        if self.ser is None:
            self.try_open_serial()
        with self.lock:
            js = self.joint_state
        lines = self.format_lines(js)
        self.send_serial(lines)

def main(args=None):
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    node = LCDDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

