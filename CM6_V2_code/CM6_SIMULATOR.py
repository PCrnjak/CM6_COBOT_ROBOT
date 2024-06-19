import customtkinter
import matplotlib.pyplot as plt
import numpy as np
import platform
import os
import logging
import random
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from s_visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
import socket
import select
import time
import re
from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS
import roboticstoolbox as rp
from spatialmath import *


#Setup IP address and Simulator port
ip = "127.0.0.1" #Loopback address
port = 5001

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))
print(f'Start listening to {ip}:{port}')

# Debug config
"""
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)
"""
#logging.disable(logging.DEBUG)

# Finds out where the program and images are stored
my_os = platform.system()
if my_os == "Windows":
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Windows")
else:
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Linux")
    
logging.debug(Image_path)

# graphical scale factor
scale_factor = 1
# robot length values (metres)
l1 = 162.65 / 1000 * scale_factor
l2 = 280 / 1000 * scale_factor
l3 = 250 / 1000 * scale_factor
l4 = 37.2 / 1000 * scale_factor

alpha_DH = [pi/2, 0, pi/2, -pi/2, pi/2, 0]

robot_tb = DHRobot(
    [
        RevoluteDH(d=l1, a=0, alpha=alpha_DH[0]),
        RevoluteDH(d=0, a=l2, alpha=alpha_DH[1]),
        RevoluteDH(d=0, a=0, alpha=alpha_DH[2],qlim = [-1.9, 3.2]),
        RevoluteDH(d=l3, a=0, alpha=alpha_DH[3]),
        RevoluteDH(d=0, a=0, alpha=alpha_DH[4]),
        RevoluteDH(d=l4, a=0, alpha=alpha_DH[5]),
    ],
    name="CM6",
)


# Elbow up config
q_t1 = np.array([-0.05177185,  0.92230595, -0.09833656, -0.01817177, -0.80872745, -0.22856314]) 

# Elbow down config
#q_t1 = np.array([-0.00929976, -0.51556136,  3.43765095,  0.20160048, -1.22769596, -0.14588157])

text_size = 14
customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

position_in_data = np.array([0, pi/2, 0, 0, 0, 0])
cart_data = np.array([0, 0, 0, 0, 0, 0])
prev_position_in_data = None
max_position_change_threshold = 0.5  # Adjust this threshold as needed



def GUI(Position_in):
    
    # This functions runs every interval of func.animation
    def show_robot(var):
        global position_in_data
        global cart_data
        global prev_position_in_data
        global q_t1
        received_data = None
        data = None
        time1 = time.perf_counter()

        """
        # Check if the elbow is up or down
        def check_elbow_configuration(robot, q):
            # Forward kinematics to get the pose of the third link
            T3 = robot.links[0].A(q[0]) * robot.links[1].A(q[1]) * robot.links[2].A(q[2])
            
            # Extract the z-axis of the orientation matrix of the third link
            z_axis = T3.R[:, 2]
            
            # Check the sign of the z component
            if z_axis[2] > 0:
                return "Elbow Down"
            else:
                return "Elbow Up"
        """  
        def check_elbow_configuration(q):

            straight_pose_j3 = pi/2
            if q[2] > straight_pose_j3:
                return "Elbow Down"
                
            elif q[2] < straight_pose_j3:
                return "Elbow Up"


        while True:
            try:
                # Check if the socket is ready to read
                ready_to_read, _, _ = select.select([sock], [], [], 0)  # Timeout of 0 second, no blocking read instantly
                #print(ready_to_read)
                # Check if there's data available to read
                if sock in ready_to_read:
                    # Receive data from the socket
                    data, addr = sock.recvfrom(1024)  # data needs to be decoded                    
                else:
                    #print(f"No sock in ready")
                    break
            except KeyboardInterrupt:
                # Handle keyboard interrupt
                break
            except Exception as e:
                # Handle other exceptions
                print(f"Error: {e}")
                break

        # Process the last received packet after exiting the loop
        if data:
            #print(data)
            received_data = data.decode()
            #print(received_data)
            # Check if the received data is in the format "pos,x,y,z,rx,ry,rz"
            if received_data.startswith("pos,"):
                pattern = re.compile(r'^pos,([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)$')
                match = pattern.match(received_data)
                
                if match:
                    # Extract position values from the matched groups
                    position_in_data = np.array([float(match.group(i)) for i in range(1, 7)])
                    # Print the updated position_in array
                    print("Received position data:", position_in_data)
                else:
                    print("Error: Invalid message format")
            # Check if the received data is in the format "cart,x,y,z"
            elif received_data.startswith("cart,"):
                # Split the message by comma and extract x, y, z values
                _, x, y, z, roll, pitch, jaw = received_data.split(",")
                cart_data = np.array([float(x), float(y), float(z), float(roll), float(pitch), float(jaw)])  #0, pi/2, pi kako mora biti; 0, 0, pi is good
                cart_data2 = np.array([float(x) *1000, float(y)*1000, float(z)*1000, float(roll), float(pitch), float(jaw)])  # Assuming rx, ry, rz are 0
                #print("Received cartesian position data:", cart_data2)
                
                 # Construct a matrix from given arguments, this will be needed pose
                Needed_pose = SE3.RPY([cart_data[3], cart_data[4], cart_data[5]], unit='rad',order='xyz')
                Needed_pose.t[0] = cart_data[0] 
                Needed_pose.t[1] =  cart_data[1]   
                Needed_pose.t[2] = cart_data[2] 

                # Calculate joint angles using ik
                prev_position_in_data = position_in_data
                q1 = robot_tb.ik_LM(Needed_pose,method = "chan",q0 = q_t1, ilimit = 25, slimit = 25)
                position_in_data = q1[0]
                elbow_config = check_elbow_configuration(position_in_data)
                #print(elbow_config)
                #print(position_in_data)
                #print(position_in_data)
                
                
                # If you want elbow down config comment this out
                if elbow_config == "Elbow Down":
                    q1 = robot_tb.ik_LM(Needed_pose,method = "chan",q0 = prev_position_in_data, ilimit = 25, slimit = 25)
                    position_in_data = q1[0]
                    print("trying again")

                if prev_position_in_data is not None:
                    position_change = np.abs(position_in_data - prev_position_in_data)
                    large_position_change_indices = np.where(position_change > max_position_change_threshold)[0]
                    if len(large_position_change_indices) > 0:
                        print("Error: Position change is too quick for joints:", large_position_change_indices)
                        print("Position change values:", position_change[large_position_change_indices])
                


            else:
                print("Error: Unknown message format")
        
        #time2 = time.perf_counter()
        #print(f"time {time2-time1} ")
        theta = np.array([0, pi/2, 0, 0, 0, 0])
        f = robot.forward(position_in_data)      
        robot.draw()

    dh_params = np.array([[l1, 0, alpha_DH[0], 0.],
                          [0., l2, alpha_DH[1], 0],
                          [0., 0, alpha_DH[2], 0],
                          [l3, 0., alpha_DH[3], 0],
                          [0., 0.,alpha_DH[4], 0.],
                          [l4, 0, alpha_DH[5], 0]])
    
    robot = RobotSerial(dh_params)

    fig = plt.figure(figsize=(7,7))
    #ax = fig.add_subplot(111, projection="3d")
    #print(plt.margins())
    #fig.margins(3,4)

    robot.init_custum_frame(fig)
    random.seed(5)
    np.set_printoptions(precision=3, suppress=True)
    theta = np.array([0., 0., -0.25 * pi, 0., 0., 0.])
    f = robot.forward(theta)

    app = customtkinter.CTk()
    app.lift()
    app.attributes('-topmost',True)
    logging.debug("Simulator running!")
        # configure window
    app.title("Simulator.py")
    app.geometry(f"{750}x{680}")
    app.wm_attributes('-topmost',False)

    # configure grid layout (4x4) 
    app.grid_columnconfigure((1,2), weight=1)
    app.grid_columnconfigure((0), weight=1)
    app.grid_rowconfigure((0), weight=0)
    app.grid_rowconfigure((1), weight=0)
    app.grid_rowconfigure((2), weight=1)
    app.grid_rowconfigure((3), weight=0) 

    Top_frame = customtkinter.CTkFrame(app ,height = 0,width=150, corner_radius=0, )
    Top_frame.grid(row=0, column=0, columnspan=4, padx=(5,5), pady=(5,5),sticky="new")
    Top_frame.grid_columnconfigure(0, weight=0)
    Top_frame.grid_rowconfigure(0, weight=0)

    def Top_frame():
            Top_frame = customtkinter.CTkFrame(app ,height = 0,width=150, corner_radius=0, )
            Top_frame.grid(row=0, column=0, columnspan=4, padx=(5,5), pady=(5,5),sticky="new")
            Top_frame.grid_columnconfigure(0, weight=0)
            Top_frame.grid_rowconfigure(0, weight=0)

        
            Control_button = customtkinter.CTkButton( Top_frame,text="Pause", width= 50,fg_color ="#313838", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command=Pause)
            Control_button.grid(row=0, column=0, padx=20,pady = (5,5),sticky="news")

            Config_button = customtkinter.CTkButton( Top_frame,text="Run", width= 50,fg_color ="#313838", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command=Run)
            Config_button.grid(row=0, column=1, padx=20,pady = (5,5),sticky="news")

            Setup_button = customtkinter.CTkButton( Top_frame,text="Sync", width= 50,fg_color ="#313838", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'))
            Setup_button.grid(row=0, column=2, padx=20,pady = (5,5),sticky="news")

    def Pause():
        ani.event_source.stop()
        
    def Run():
        ani.event_source.start()
    
    Top_frame()

    canvas = FigureCanvasTkAgg(fig, master=app)
    canvas.draw()
    canvas.get_tk_widget().grid(row=1, column=1, padx=20,pady = (5,5),sticky="news")

    ani = animation.FuncAnimation(fig, show_robot, interval=85,frames=30)

    app.mainloop() 


if __name__ == "__main__":

    Position_in = [0,0,0,0,0,0]
    GUI(Position_in)
