import Spectral_BLDC as Spectral
import SourceRoboticsToolbox
import time
import socket
import numpy as np
import keyboard

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

# Initialize position values
position_values = np.array([0, np.pi/2, 0, 0, 0, 0])
received_ids = [0,0,0,0,0,0]
active_command = 'i'
active_hold = np.array([0, np.pi/2, 0, 0, 0, 0])

while True:

    # Check for keyboard input
    if keyboard.is_pressed('i'):
        print("You pressed 'i'")
        active_command = 'i'
    elif keyboard.is_pressed('c'):
        print("You pressed 'c'")
        active_command = 'c'
    elif keyboard.is_pressed('p'):
        print("You pressed 'p'")
        active_command = 'p'
        active_hold = np.copy(position_values)
    elif keyboard.is_pressed('d'):
        print("You pressed 'd'")
        active_command = 'd'
    elif keyboard.is_pressed('e'):
        print("You pressed 'e'")
        active_command = 'e'
    elif keyboard.is_pressed('f'):
        print("You pressed 'f'")
        active_command = 'f'

    if active_command == 'i':
        for motor in Motor:
            motor.Send_Respond_Encoder_data()

    elif active_command == 'c':
        for motor in Motor:
            motor.Send_Clear_Error()

    elif active_command == 'd':
        Motor[0].Send_PD_Gains(0.49,0.01)
        Motor[1].Send_PD_Gains(0.49,0.01)
        Motor[2].Send_PD_Gains(0.49,0.01)
        Motor[3].Send_PD_Gains(0.49,0.01)
        Motor[4].Send_PD_Gains(0.49,0.01)
        #Motor[5].Send_Respond_Encoder_data()
        Motor[5].Send_PD_Gains(0.49,0.01)

    elif active_command == 'e':
        Motor[0].Send_PD_Gains(0.14,0.0028)
        Motor[1].Send_PD_Gains(0.14,0.0028)
        Motor[2].Send_PD_Gains(0.14,0.0028)
        Motor[3].Send_PD_Gains(0.14,0.0028)
        Motor[4].Send_PD_Gains(0.14,0.0028)
        #Motor[5].Send_Respond_Encoder_data()
        Motor[5].Send_PD_Gains(0.14,0.028)

    elif active_command == 'f':
        Motor[0].Send_PD_Gains(0.49,0.028)
        Motor[1].Send_PD_Gains(0.49,0.028)
        Motor[2].Send_PD_Gains(0.49,0.028)
        Motor[3].Send_PD_Gains(0.49,0.028)
        Motor[4].Send_PD_Gains(0.49,0.028)
        #Motor[5].Send_Respond_Encoder_data()
        Motor[5].Send_PD_Gains(0.49,0.028)

    elif active_command == 'p':

        #Motor[0].Send_Respond_Encoder_data()
        #Motor[1].Send_Respond_Encoder_data()
        #Motor[2].Send_Respond_Encoder_data()
        #Motor[3].Send_Respond_Encoder_data()
        #Motor[4].Send_Respond_Encoder_data()
        #Motor[5].Send_Respond_Encoder_data()

        tick = Joint[0].get_encoder_position(active_hold[0])
        Motor[0].Send_data_pack_PD(tick,0,0)
        tick = Joint[1].get_encoder_position(active_hold[1])
        Motor[1].Send_data_pack_PD(tick,0,0)
        tick = Joint[2].get_encoder_position(active_hold[2])
        Motor[2].Send_data_pack_PD(tick,0,0)
        tick = Joint[3].get_encoder_position(active_hold[3])
        Motor[3].Send_data_pack_PD(tick,0,0)
        tick = Joint[4].get_encoder_position(active_hold[4])
        Motor[4].Send_data_pack_PD(tick,0,0)
        tick = Joint[5].get_encoder_position(active_hold[5])
        Motor[5].Send_data_pack_PD(tick,0,0)

    for i in range(1, 9):  # Loop 9-1=8 to check for received data
        message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
        print(f"unpack{i} is: {UnpackedMessageID}")

        # Check if UnpackedMessageID is not None 
        if UnpackedMessageID is not None:
            
            # Update received id index; meaning that we received response from that CAN ID
            received_ids[UnpackedMessageID[0]] = 1
            Motor[UnpackedMessageID[0]].UnpackData(message,UnpackedMessageID)
            print(f"Motor {UnpackedMessageID[0]}, position is: {Motor[UnpackedMessageID[0]].position}")
            unwrapped_position_raw = Joint[UnpackedMessageID[0]].unwrap_position(Motor[UnpackedMessageID[0]].position)
            position_values[UnpackedMessageID[0]] =  Joint[UnpackedMessageID[0]].get_joint_position(Motor[UnpackedMessageID[0]].position)


            if initial_setup[UnpackedMessageID[0]] == 0:
                initial_setup[UnpackedMessageID[0]] = 1
                Joint[UnpackedMessageID[0]].determine_sector(Motor[UnpackedMessageID[0]].position)

    print(position_values)
    print(f"active commadn is : {active_command}")
    print(f"active hold is: {active_hold}")
    position_values_str = ','.join(map(str, position_values))
    msg = f"pos,{position_values_str}"  # Include "pos" at the beginning
    msg_bytes = msg.encode()
    sock.sendto(msg_bytes, (ip, port))
    
    time.sleep(0.05)
