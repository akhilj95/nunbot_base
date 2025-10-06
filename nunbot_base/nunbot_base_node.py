import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sdpo_drivers_interfaces.msg import MotEnc, MotEncArray
import serial
import math
from collections import deque

# Rpi 5 gpio
# Run sudo apt install python3-libgpiod
import gpiod

class NunbotBase(Node):

    def __init__(self):
        super().__init__('nunbot_base')

        # parameters for serial communication and robot speed
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 3.14)
        self.declare_parameter('ticks_per_rev', 2880.0)  # Wheel encoder counts per revolution

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().double_value

        # LOW Voltage threshold for batteries
        self.low_voltage_threshold = 13  # Volts

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Serial port {port} opened at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            rclpy.shutdown()
            return

        # subscribers and publishers
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.mot_enc_pub = self.create_publisher(MotEncArray, '/nunbot/motors_enc', 25)
        self.ir_dist_pub_0 = self.create_publisher(Range, '/nunbot/ir_distance_0', 10)
        self.ir_dist_pub_1 = self.create_publisher(Range, '/nunbot/ir_distance_1', 10)
        self.voltage_pub = self.create_publisher(Float32MultiArray, '/nunbot/voltage', 10)
        self.button_pairing_pub = self.create_publisher(Bool, '/nunbot/button_pairing', 10)
        self.button_onoff_pub = self.create_publisher(Bool, '/nunbot/button_onoff', 10)

        self.last_time = self.get_clock().now()

        # start timer to read serial data at 20 Hz
        self.create_timer(0.05, self.read_serial)

        # Voltage smoothing
        window_len = 6
        # buffer initialized with battery nominal volatage to be safe
        self.voltage_buffers = [
            deque([14.8] * window_len, maxlen=window_len),
            deque([14.8] * window_len, maxlen=window_len)
        ]

        # Define GPIO pins for LEDs
        self.BATTERY1_LED_PIN = 17
        self.BATTERY2_LED_PIN = 27

        # Initialize GPIO with graceful degradation
        self.led1_line = None
        self.led2_line = None
        GPIO_CHIP_PATH = '/dev/gpiochip4'
        
        try:
            # gpiod 2.x API - request_lines function
            self.led1_line = gpiod.request_lines(
                GPIO_CHIP_PATH,
                config={
                    self.BATTERY1_LED_PIN: gpiod.LineSettings(
                        direction=gpiod.line.Direction.OUTPUT
                    )
                },
                consumer="battery1_led"
            )
            self.led2_line = gpiod.request_lines(
                GPIO_CHIP_PATH,
                config={
                    self.BATTERY2_LED_PIN: gpiod.LineSettings(
                        direction=gpiod.line.Direction.OUTPUT
                    )
                },
                consumer="battery2_led"
            )          
            self.get_logger().info("GPIO initialized successfully for battery LEDs")
        except Exception as e:
            self.get_logger().warning(f"GPIO initialization failed: {e}. LED control disabled, continuing without GPIO.")
            self.led1_line = None
            self.led2_line = None

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Compute direction in degrees from linear x,y velocities
        direction_rad = math.atan2(linear_y, linear_x)
        direction_deg = int(math.degrees(direction_rad))
        if direction_deg < 0:
            direction_deg += 360

        # Linear speed magnitude scaled to percent
        linear_speed = math.sqrt(linear_x**2 + linear_y**2)
        linear_pct = int(min((linear_speed / self.max_linear_speed) * 100, 100))
        rot_pct = int(max(min((angular_z / self.max_angular_speed) * 100, 100), -100))

        command = f"[LIN:{linear_pct},ROT:{rot_pct},DIR:{direction_deg}]\n"
        try:
            self.ser.write(command.encode())
            self.get_logger().debug(f"Sending command: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_serial(self):
        buffer = getattr(self, '_serial_buffer', '')  # Use persistent buffer stored in self
        try:
            while self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.parse_and_publish(line)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
        finally:
            self._serial_buffer = buffer  # Save leftover partial line for next call

    def parse_and_publish(self, line):
        # Expected line format
        # DEN:deltaEnc1,deltaEnc2,deltaEnc3;VEL:vel1,vel2,vel3;IRD:val0,val1;VOL:val0,val1;BTN:pair,onoff
        try:
            parts = line.split(';')
            data = {}
            for p in parts:
                if ':' not in p:
                    self.get_logger().warning(f"Skipping invalid segment (no key:value): {p}")
                    continue  # Skip or handle differently if needed
                key, vals = p.split(':', 1)
                data[key] = vals.split(',')

            # Now check if 'DEN' key is actually present before parsing values
            # Add validation for all required keys
            required_keys = ['DEN', 'VEL', 'IRD', 'VOL', 'BTN']
            missing_keys = [k for k in required_keys if k not in data]
            if missing_keys:
                self.get_logger().warning(f"Missing keys {missing_keys} in line: {line}")
                return

            # Parse delta encoder values
            deltaEnc = list(map(float, data['DEN']))

            # Parse wheel angular velocities in rad/sec
            vel= list(map(float, data['VEL']))

            current_time = self.get_clock().now()
  
            # Publish Motor data for sdpo_localization_odom
            mot_enc_array_msg = MotEncArray()
            mot_enc_array_msg.stamp = current_time.to_msg()
            mot_enc_array_msg.mot_enc = []

            for i in range(3):
                mot_enc_msg = MotEnc()
                mot_enc_msg.enc_delta = int(deltaEnc[i])
                mot_enc_msg.ticks_per_rev = self.ticks_per_rev
                mot_enc_msg.ang_speed = vel[i]
                mot_enc_array_msg.mot_enc.append(mot_enc_msg)

            self.mot_enc_pub.publish(mot_enc_array_msg)

            # Publish IR distance as Range messages for 2 sensors
            for i in range(2):
                dist_msg = Range()
                dist_msg.header.stamp = current_time.to_msg()
                dist_msg.header.frame_id = f'ir_distance_{i}_frame'
                dist_msg.radiation_type = Range.INFRARED
                dist_msg.field_of_view = 0.1  # adjust for your sensor
                dist_msg.min_range = 0.02
                dist_msg.max_range = 0.3
                dist_msg.range = float(data['IRD'][i])
                if i == 0:
                    self.ir_dist_pub_0.publish(dist_msg)
                else:
                    self.ir_dist_pub_1.publish(dist_msg)

            # Checking battery voltages
            voltage1 = float(data['VOL'][0])
            voltage2 = float(data['VOL'][1])

            self.voltage_buffers[0].append(voltage1)
            self.voltage_buffers[1].append(voltage2)

            smoothed_voltage = [
                sum(self.voltage_buffers[0]) / len(self.voltage_buffers[0]),
                sum(self.voltage_buffers[1]) / len(self.voltage_buffers[1])
            ]

            # Publish voltage readings
            voltage_msg = Float32MultiArray()
            voltage_msg.data = smoothed_voltage
            self.voltage_pub.publish(voltage_msg)

            # Turn on LED if voltage is above threshold (only if GPIO is available)
            if self.led1_line is not None and self.led2_line is not None:
                self.led1_line.set_value(
                    self.BATTERY1_LED_PIN, 
                    gpiod.line.Value(1) if smoothed_voltage[0] < self.low_voltage_threshold else gpiod.line.Value(0)
                )
                self.led2_line.set_value(
                    self.BATTERY2_LED_PIN, 
                    gpiod.line.Value(1) if smoothed_voltage[1] < self.low_voltage_threshold else gpiod.line.Value(0)
                )
                # self.get_logger().info(f"Voltages: {smoothed_voltage[0]} {smoothed_voltage[1]}")

            # Publish button states
            self.button_pairing_pub.publish(Bool(data=bool(int(data['BTN'][0]))))
            self.button_onoff_pub.publish(Bool(data=bool(int(data['BTN'][1]))))

        except Exception as e:
            self.get_logger().warning(f"Failed to parse/publish line: {line} error: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    node = NunbotBase()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    finally:
        if hasattr(node, "ser") and node.ser:
            node.ser.close()
        
        # Release GPIO lines individually
        if hasattr(node, "led1_line") and node.led1_line is not None:
            try:
                node.led1_line.release()
            except Exception as e:
                node.get_logger().warning(f"Error releasing LED1: {e}")
        
        if hasattr(node, "led2_line") and node.led2_line is not None:
            try:
                node.led2_line.release()
            except Exception as e:
                node.get_logger().warning(f"Error releasing LED2: {e}")

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
