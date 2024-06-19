import Spectral_BLDC as Spectral
import SourceRoboticsToolbox
import time
import socket


# Sender configuration
ip = "127.0.0.1" #Loopback address
port = 5001
# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


Master_position = [11800,6910,770,3255,9477,5487]
Joint_reduction_ratio = [8, 8, 9.142857143, 6.5, 6, 5] # Reduction ratio we have on our joints

Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)

Motor = []

Motor.append(Spectral.SpectralCAN(node_id=0, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=1, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=2, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=3, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=4, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=5, communication=Communication1))

Joint = []

Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[0], gear_ratio = Joint_reduction_ratio[0], offset = 0, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[1], gear_ratio = Joint_reduction_ratio[1], offset = 3.512, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[2], gear_ratio = Joint_reduction_ratio[2], offset = -1.265, dir = 1))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[3], gear_ratio = Joint_reduction_ratio[3], offset = 0, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[4], gear_ratio = Joint_reduction_ratio[4], offset = 0, dir = 1))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[5], gear_ratio = Joint_reduction_ratio[5], offset = 0, dir = 0))

timeout_setting = 0.001

initial = 0
initial_setup = [0,0,0,0,0,0]

testing = 5

Joint_positions = [0,0,0,0,0,0]

while True:

    Motor[testing].Send_Respond_Encoder_data()
    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
    if message is not None:

        Motor[testing].UnpackData(message,UnpackedMessageID)
        unwrapped_position_raw = Joint[testing].unwrap_position(Motor[testing].position)
        print(f"Motor6 position is: {Motor[testing].position} motor unwrapped {unwrapped_position_raw}")
        True_position = Joint[testing].get_joint_position(Motor[testing].position)
        print(True_position)
        #tick = Joint[testing].get_encoder_position(True_position)
        #print(tick )

        if initial == 0:
            initial = 1
            Joint[testing].determine_sector(Motor[testing].position)

    else:
        print(f"No message after timeout period mot1! number: ")

    # Convert position value and send time to a string separated by a comma
    msg = f"{100}".encode()
    
    sock.sendto(msg, (ip, port))
    
    time.sleep(1)
