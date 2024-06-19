import Spectral_BLDC as Spectral
import SourceRoboticsToolbox
import time
import psutil
import os


Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor6 = Spectral.SpectralCAN(node_id=5, communication=Communication1)
Joint = SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=6000, gear_ratio = 5, offset = 0, dir = 0)


timeout_setting = 0.001

initial = 0
True_position = 0


while True:

    Motor6.Send_Respond_Encoder_data()
    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
    if message is not None:

        Motor6.UnpackData(message,UnpackedMessageID)
        unwrapped_position_raw = Joint.unwrap_position(Motor6.position)
        print(f"Motor6 position is: {Motor6.position} motor unwrapped {unwrapped_position_raw}")

        True_position = Joint.get_joint_position(Motor6.position)
        print(True_position)
        #Joint.print_offset_ticks()
        tick = Joint.get_encoder_position(True_position)
        print(tick )

    else:
        print(f"No message after timeout period mot1! number: ")
    
    if initial == 0:
        initial = 1
        Joint.determine_sector(Motor6.position)

    time.sleep(1)
