import os
import json
import argparse

from pathlib import Path

from multiprocessing import Process
from multiprocessing import Manager
from multiprocessing import Pipe

from mult_packets_pick_place import main_multi_packets
# from tracking_program import tracking_program
from robot_control import RobotControl
from robot_communication import RobotCommunication


if __name__ == "__main__":
    rob_config = None

    

    # Create dictionary with program modes, functions and robot positions they use
    modes_dict = {
        "1": {
            "help": "Object sorting with moving conveyor",
            "dict": robot_poses["short_pick_place_dict"],
            "func": main_multi_packets,
        },
        "2": {
            "help": "Tracking packet with robot gripper (Does not work)",
            "dict": robot_poses["short_pick_place_dict"],
            "func": None,
        },
    }

    # Initialize robot demos and robot control objects
    r_control = RobotControl(None)
    r_comm_info = RobotCommunication()

    # Start program mode selection
    while True:
        # Read mode input
        if rob_config.mode in modes_dict:
            mode = rob_config.mode
            rob_config.mode = "0"
        else:
            # Show message about robot programs
            print("Select pick and place mode:")
            for mode in modes_dict:
                print(f"{mode} : {modes_dict[mode]['help']}")
            print("e : Exit")
            mode = input()

        # If mode is a program key
        if mode in modes_dict:
            # Set robot positions and robot program
            r_control.rob_dict = modes_dict[mode]["dict"]
            robot_prog = modes_dict[mode]["func"]
            print(f"[INFO] Starting mode {mode} ({modes_dict[mode]['help']})")

            with Manager() as manager:
                # Dictionary Manager for passing data between processes
                manag_info_dict = manager.dict()

                # Value Manager separate from dictionary for passing encoder value between processes
                # Encoder value needs to be updated more often
                manag_encoder_val = manager.Value("d", None)

                # Pipe objects for passing commands from main to robot server
                # One object goes into main, the other into the server
                # Pipe is buffered, which makes it better than Manager in this case
                pipe_main, pipe_server = Pipe()

                # Create and start processes
                main_proc = Process(
                    target=robot_prog,
                    args=(
                        rob_config,
                        r_control.rob_dict,
                        manag_info_dict,
                        manag_encoder_val,
                        pipe_main,
                    ),
                )
                info_server_proc = Process(
                    target=r_comm_info.robot_server,
                    args=(manag_info_dict, manag_encoder_val),
                )
                control_server_proc = Process(
                    target=r_control.control_server,
                    args=(pipe_server,),
                )

                main_proc.start()
                info_server_proc.start()
                control_server_proc.start()

                # Wait for the main process to end
                main_proc.join()
                info_server_proc.kill()
                control_server_proc.kill()

            if rob_config.auto_exit:
                exit()

        # If input is exit, exit python
        elif mode == "e":
            exit()
