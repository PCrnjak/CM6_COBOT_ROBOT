import Spectral_BLDC as Spectral
import time
import numpy as np

# Set the CPU affinity to CPU core 0


Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor = []

Motor.append(Spectral.SpectralCAN(node_id=0, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=1, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=2, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=3, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=4, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=5, communication=Communication1))

timeout_setting = 0.00005
received_ids = [0,0,0,0,0,0]

# Initialize position values
position_values = np.array([0, 0, 0, 0, 0, 0])

while True:

    time1 = time.perf_counter()
    # Send data to all motors
    Motor[0].Send_Respond_Encoder_data()
    Motor[1].Send_Respond_Encoder_data()
    Motor[2].Send_Respond_Encoder_data()
    Motor[3].Send_Respond_Encoder_data()
    Motor[4].Send_Respond_Encoder_data()
    Motor[5].Send_Respond_Encoder_data()


    for i in range(1, 9):  # Loop 9-1=8 to check for received data
        message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
        print(f"unpack{i} is: {UnpackedMessageID}")

        # Check if UnpackedMessageID is not None 
        if UnpackedMessageID is not None:
            
            # Update received id index; meaning that we received response from that CAN ID
            received_ids[UnpackedMessageID[0]] = 1
            Motor[UnpackedMessageID[0]].UnpackData(message,UnpackedMessageID)
            print(f"Motor {UnpackedMessageID[0]}, position is: {Motor[UnpackedMessageID[0]].position}")
            position_values[UnpackedMessageID[0]] = Motor[UnpackedMessageID[0]].position

    """
    print("Received IDs:", received_ids)
    if all(count == 1 for count in received_ids):
        print("All indexes are equal to 1")
    else:
        # Find indices not equal to 1
        indices_not_equal_to_1 = [i for i, count in enumerate(received_ids) if count != 1]
        if indices_not_equal_to_1:
            print("Indexes not equal to 1:", indices_not_equal_to_1)
            # Send data packs for indices not equal to 1 and try to receive data
            for index in indices_not_equal_to_1:
                Motor[index].Send_Respond_Encoder_data()
                # Try to receive data after sending data
                message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
                if UnpackedMessageID is not None:
                    print(f"Received data after sending to Motor {index + 1}: {UnpackedMessageID}")
                    Motor[UnpackedMessageID[0]].UnpackData(message,UnpackedMessageID)
                    print(f"Motor {UnpackedMessageID[0]}, position is: {Motor[UnpackedMessageID[0]].position}")
    """
    print(position_values)
    received_ids = [0,0,0,0,0,0]

    time2 = time.perf_counter()
    print(f"Total time is: {time2-time1}")

    time.sleep(1)