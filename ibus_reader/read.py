import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def deadzone(raw_value : int, center : int = 1500, deadzone_value : int = 100):
    if(abs(raw_value - center) <= deadzone_value):
        return center
    else:
        return raw_value



class IBusReader(Node):
    
    
    def __init__(self):

        super().__init__('ibus_reader')

        # ibus param, default to AMA0, can be changed through launch args
        self.declare_parameter("ibus_channel", "/dev/ttyAMA0")
        ibus_param = self.get_parameter("ibus_channel")

        self.pub_1 = self.create_publisher(Int32, 'ch1', 1)
        self.pub_2 = self.create_publisher(Int32, 'ch2', 1)
        self.pub_3 = self.create_publisher(Int32, 'ch3', 1)
        self.pub_4 = self.create_publisher(Int32, 'ch4', 1)
        self.pub_5 = self.create_publisher(Int32, 'ch5', 1)
        self.pub_6 = self.create_publisher(Int32, 'ch6', 1)
        self.pub_7 = self.create_publisher(Int32, 'ch7', 1)
        self.pub_8 = self.create_publisher(Int32, 'ch8', 1)
        
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = 0
        self.ch5 = 0
        self.ch6 = 0
        self.ch7 = 0
        self.ch8 = 0
        
        serial_connected = False
        while not serial_connected:
            try:
                self.ser = serial.Serial(ibus_param.value, 115200)
                serial_connected = True
            except serial.SerialException:
                self.get_logger().log(f"Waiting for {ibus_param.value}...", 20)
                
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.timer = self.create_timer(0.001, self.timer_callback)


    def read_serial_data(self):
        
        frame = bytearray()
        received_data = self.ser.read()  # Read serial port
        int_received = int.from_bytes(received_data, byteorder='little')

        if int_received == 32:
            frame.extend(received_data)  # Add the header
            next_bytes = self.ser.read(31)  # Read the next 31 bytes
            frame.extend(next_bytes)  # Add the read bytes to the frame bytearray

            ch1byte = bytearray()
            ch1byte.append(frame[2])
            ch1byte.append(frame[3])
            self.ch1 = int.from_bytes(ch1byte, byteorder='little')
            
            ch2byte = bytearray()
            ch2byte.append(frame[4])
            ch2byte.append(frame[5])
            self.ch2 = int.from_bytes(ch2byte, byteorder='little')

            ch3byte = bytearray()
            ch3byte.append(frame[6])
            ch3byte.append(frame[7])
            self.ch3 = int.from_bytes(ch3byte, byteorder='little')

            ch4byte = bytearray()
            ch4byte.append(frame[8])
            ch4byte.append(frame[9])
            self.ch4 = int.from_bytes(ch4byte, byteorder='little')

            ch5byte = bytearray()
            ch5byte.append(frame[10])
            ch5byte.append(frame[11])
            self.ch5 = int.from_bytes(ch5byte, byteorder='little')

            ch6byte = bytearray()
            ch6byte.append(frame[12])
            ch6byte.append(frame[13])
            self.ch6 = int.from_bytes(ch6byte, byteorder='little')

            ch7byte = bytearray()
            ch7byte.append(frame[14])
            ch7byte.append(frame[15])
            self.ch7 = int.from_bytes(ch7byte, byteorder='little')

            ch8byte = bytearray()
            ch8byte.append(frame[16])
            ch8byte.append(frame[17])
            self.ch8 = int.from_bytes(ch8byte, byteorder='little')

    def channel_output(self):
        print("ch1=", self.angular_x, "ch2=", self.angular_y, "ch3=", self.linear_z, "ch4=", self.angular_z, 
            "ch5=", self.sw_a, "ch6=", self.sw_d, "ch7=", self.sw_c, "ch8=", self.sw_b)
    
    def timer_callback(self):
        self.read_serial_data()
        
        msg = Int32()
        msg.data = deadzone(self.ch1, deadzone_value=50)
        self.pub_1.publish(msg)
        msg.data = deadzone(self.ch2)
        self.pub_2.publish(msg)
        msg.data = deadzone(self.ch3)
        self.pub_3.publish(msg)
        msg.data = deadzone(self.ch4 ,deadzone_value=50)
        self.pub_4.publish(msg)
        msg.data = self.ch5
        self.pub_5.publish(msg)
        msg.data = self.ch6
        self.pub_6.publish(msg)
        msg.data = self.ch7
        self.pub_7.publish(msg)
        msg.data = self.ch8
        self.pub_8.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    reader = IBusReader()
    rclpy.spin(reader)
    reader.destroy_node()
    rclpy.shutdown()
    
    
# Testing block
if __name__ == "__main__":
    main()
    
    