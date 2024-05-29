import serial
import struct
import time
import select
import threading
import json 
SERIAL_PORT = '/dev/ttyUSB0'  # Update to your Arduino's serial port
SERIAL_PORT1 = '/dev/ttyTHS1'  # Update to your Arduino's serial port
SERIAL_BAUD_RATE = 256000
SERIAL_BAUD_RATE1 = 256000


PACKET_START_BYTE = 0xAA
PACKET_STOP_BYTE = 0x55
PACKET_HEADER_BYTE = 0xBB
flag=False
data_to_esp = None
low_side_flag=False
# Define the structure format for ControlData
# ? for bool, B for uint8_t
CONTROL_DATA_FORMAT = '? B B B B B B ? ? ? B B B B B'
CONTROL_DATA_SIZE = struct.calcsize(CONTROL_DATA_FORMAT)

# Define the structure format for CustomPacket
# Start byte, Header byte, ControlData, CRC byte, Stop byte
PACKET_FORMAT = f'B B {CONTROL_DATA_SIZE}s B B'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

class ControlData:
    def __init__(self):
        self.grasp_command = False
        self.set_velocity = 0
        self.stroke_min = 0
        self.stroke_max = 0
        self.set_max_contact_force = 0
        self.set_axial_force_threshold = 0
        self.set_lateral_force_threshold = 0
        self.grasp_status = False
        self.axial_force_threshold_status = False
        self.lateral_force_threshold_status = False
        self.current_stroke_length = 0
        self.current_sensor = 0
        self.joint_torque_estimation = 0
        self.axial_force_feedback = 0
        self.lateral_force_feedback = 0

class CustomPacket:
    def __init__(self):
        self.start = 0
        self.header = 0
        self.data = bytearray()
        self.crc = bytearray(1)
        self.stop = 0

def calculate_crc8(data):
    crc = 0x00
    polynomial = 0x8C
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
    return crc & 0xFF

def calculate_crc8_1(data, length):
    crc = 0x00  # Initial value
    polynomial = 0x8C  # CRC-8 polynomial

    for i in range(length):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1

    return crc
def pack_data(data, packet):
    packet.start = PACKET_START_BYTE
    packet.header = PACKET_HEADER_BYTE
    packet.data = bytearray([
        data.grasp_command,
        data.set_velocity,
        data.stroke_min,
        data.stroke_max,
        data.set_max_contact_force,
        data.set_axial_force_threshold,
        data.set_lateral_force_threshold,
        data.grasp_status,
        data.axial_force_threshold_status,
        data.lateral_force_threshold_status,
        data.current_stroke_length,
        data.current_sensor,
        data.joint_torque_estimation,
        data.axial_force_feedback,
        data.lateral_force_feedback
    ])
    crc = calculate_crc8_1(packet.data, len(packet.data))
    packet.crc[0] = crc & 0xFF  # Store CRC directly as one byte
    packet.stop = PACKET_STOP_BYTE

def send_packet(packet, serial_port):
    serial_port.write(bytes([packet.start]))
    serial_port.write(bytes([packet.header]))
    serial_port.write(packet.data)
    serial_port.write(packet.crc)
    serial_port.write(bytes([packet.stop]))


def read_packet(serial_port):
    data = serial_port.read(PACKET_SIZE)
    print(data)
    if len(data) == PACKET_SIZE:
        packet = struct.unpack(PACKET_FORMAT, data)
        start, header, control_data_bytes, crc, stop = packet

        if start != PACKET_START_BYTE or header != PACKET_HEADER_BYTE or stop != PACKET_STOP_BYTE:
            print("Packet start/header/stop byte mismatch")
            serial_port.flush()
            return None

        # Verify CRC
        calculated_crc = calculate_crc8(control_data_bytes)
        if calculated_crc != crc:
            print("CRC mismatch")
            serial_port.flush()
            return None

        control_data = struct.unpack(CONTROL_DATA_FORMAT, control_data_bytes)
        return control_data
    return None
control_data = ControlData()
packet_ = CustomPacket()



time_s = time.time()
def write_serial():
    global time_s,packet,flag,low_side_flag
    while True:
        if time.time() - time_s >= 0.03:
            if not low_side_flag and not flag:
                serial_port.write(bytes([128]))
                low_side_flag=True
            elif flag:
                serial_port.write(bytes([129]))
                
                serial_port.write(data_to_esp)
                flag=False

            time_s = time.time()



def read_serial():
    global control_data,low_side_flag
    with serial.Serial(SERIAL_PORT, SERIAL_BAUD_RATE, timeout=1) as ser:
        while True:
            if low_side_flag:
                packet = read_packet(ser)
                low_side_flag=False
                if packet:
                    (
                        grasp_command,
                        set_velocity,
                        stroke_min,
                        stroke_max,
                        set_max_contact_force,
                        set_axial_force_threshold,
                        set_lateral_force_threshold,
                        grasp_status,
                        axial_force_threshold_status,
                        lateral_force_threshold_status,
                        current_stroke_length,
                        current_sensor,
                        joint_torque_estimation,
                        axial_force_feedback,
                        lateral_force_feedback
                    ) = packet
                    # print(f"Grasp Command: {grasp_command}")
                    # print(f"Set Velocity: {set_velocity}")
                    # print(f"Stroke Min: {stroke_min}")
                    # print(f"Stroke Max: {stroke_max}")
                    # print(f"Set Max Contact Force: {set_max_contact_force}")
                    # print(f"Set Axial Force Threshold: {set_axial_force_threshold}")
                    # print(f"Set Lateral Force Threshold: {set_lateral_force_threshold}")
                    # print(f"Grasp Status: {grasp_status}")
                    # print(f"Axial Force Threshold Status: {axial_force_threshold_status}")
                    # print(f"Lateral Force Threshold Status: {lateral_force_threshold_status}")
                    # print(f"Current Stroke Length: {current_stroke_length}")
                    # print(f"Current Sensor: {current_sensor}")
                    # print(f"Joint Torque Estimation: {joint_torque_estimation}")
                    # print(f"Axial Force Feedback: {axial_force_feedback}")
                    # print(f"Lateral Force Feedback: {lateral_force_feedback}")
                    
                control_data.grasp_status = grasp_command
                control_data.set_velocity = set_velocity
                control_data.stroke_min = stroke_min 
                control_data.stroke_max = stroke_max 
                control_data.set_max_contact_force = set_max_contact_force  
                control_data.set_axial_force_threshold = set_axial_force_threshold 
                control_data.set_lateral_force_threshold = set_lateral_force_threshold 
                control_data.grasp_status = grasp_status
                control_data.axial_force_threshold_status = axial_force_threshold_status
                control_data.lateral_force_threshold_status = lateral_force_threshold_status
                control_data.current_stroke_length = current_stroke_length 
                control_data.current_sensor = current_sensor 
                control_data.joint_torque_estimation = joint_torque_estimation 
                control_data.axial_force_feedback = axial_force_feedback 
                control_data.lateral_force_feedback = lateral_force_feedback 

time_s1=time.time()
def write_serial1():
    global time_s1,packet_,control_data,low_side_flag
    while True:
        if low_side_flag:
            # print("KEE:")
            pack_data(control_data, packet_)
            send_packet(packet_, serial_port1)
            time_s1 = time.time()

def read_serial1():
    global flag,data_to_esp
    while True:
        ready_to_read, _, _ = select.select([serial_port1], [], [], 0.01)
        
        if ready_to_read:
            received_data = serial_port1.read(serial_port1.inWaiting()).decode().strip()
            
            print("Received:", received_data)
            data_dict = json.loads(received_data)
            if data_dict.get("graspCommand") == str(True) :
                control_data.grasp_command=True
            elif data_dict.get("graspCommand") == str(False):
                control_data.grasp_command=False
            control_data.stroke_min=int(data_dict.get("strokeMin"))
            control_data.stroke_max=int(data_dict.get("strokeMax"))
            control_data.set_max_contact_force=int(data_dict.get("setMaxContactForce"))
            control_data.set_velocity=int(data_dict.get("setVelocity"))
            control_data.set_axial_force_threshold=int(data_dict.get("setAxialForceThreshold"))
            control_data.set_lateral_force_threshold=int(data_dict.get("setLateralForceThreshold"))
            
            # data_to_esp = bytearray([
            #         control_data.grasp_command,
            #         control_data.stroke_min])
            # print(data_to_esp)
            
            data_to_esp = struct.pack('?bbbbbb',control_data.grasp_command,  control_data.stroke_min,control_data.stroke_max, control_data.set_max_contact_force, control_data.set_velocity, control_data.set_axial_force_threshold, control_data.set_lateral_force_threshold)
            print(data_to_esp)
            
            # serial_port.write(data_to_esp)
            
            flag=True
        else:
            
            
            flag=False


serial_port = serial.Serial(
    port=SERIAL_PORT,
    baudrate=SERIAL_BAUD_RATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,  # Correct placement of timeout inside the parentheses
)
serial_port1 = serial.Serial(
    port=SERIAL_PORT1,
    baudrate=SERIAL_BAUD_RATE1,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,  # Correct placement of timeout inside the parentheses
)



read_thread = threading.Thread(target=read_serial)
read_thread1 = threading.Thread(target=read_serial1)
write_thread = threading.Thread(target=write_serial)
write_thread1 = threading.Thread(target=write_serial1)

read_thread.start()
read_thread1.start()
write_thread.start()
write_thread1.start()

try:
    read_thread.join()
    read_thread1.join()
    write_thread.join()
    write_thread1.join()
except KeyboardInterrupt:
    print("Exiting Program")
except Exception as exception_error:
    print("Error occurred:", str(exception_error))

finally:
    serial_port.close()
    serial_port1.close()
