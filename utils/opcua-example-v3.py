import asyncua.sync as opcua
import asyncua.ua as uatype
import logging
import time

# Set the logging level to only print errors, affects the opcua library loggers
logging.basicConfig(level=logging.ERROR)

# Create client and connect to the server
PLC_OPCUA_URL = 'opc.tcp://10.100.0.120:4840/'
opcua_client = opcua.Client(PLC_OPCUA_URL)
opcua_client.connect()

# Read user defined data types from server
# Accesible as uatype.TYPE_NAME
types = opcua_client.load_data_type_definitions()

############################################
# Nodes are sorted into groups of namespaces
# Every namepsace is identified with a string URI and numeric index
# Index can change, so it needs to be obtained from the server every time after connecting, using the URI
cell_ns_uri = "http://pickplace.ti40.cz"
robot_ns_uri = "http://robots.pickplace.ti40.cz"
conveyor_ns_uri = "http://conveyor.pickplace.ti40.cz"
grippers_ns_uri = "http://grippers.pickplace.ti40.cz"
cell_ns_idx = opcua_client.get_namespace_index(cell_ns_uri)
robot_ns_idx = opcua_client.get_namespace_index(robot_ns_uri)
conveyor_ns_idx = opcua_client.get_namespace_index(conveyor_ns_uri)
grippers_ns_idx = opcua_client.get_namespace_index(grippers_ns_uri)

########################################
# Read nodes from server using node IDs:
# Robot nodes:
node_robot = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32')
node_robot_coom_err = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.CommunicationError') # Bool
node_robot_powered_on = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.PoweredON') # Bool
node_robot_busy = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.Busy') # Bool
node_robot_is_homed = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.IsHomed') # Bool
node_robot_position = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.Position') # POS6D
node_robot_position_valid = opcua_client.get_node(f'ns={robot_ns_idx};s=Cell.Robots.R32.Status.PositionValid') # Bool
# Gripper nodes:
node_gripper = opcua_client.get_node(f'ns={grippers_ns_idx};s=Cell.Grippers.G1')
node_gripper_active = opcua_client.get_node(f'ns={grippers_ns_idx};s=Cell.Grippers.G1.Status.Active') # Bool
# Conveyor nodes:
node_conveyor = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor')
node_conveyor_moving = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor.Status.Moving') # Bool
node_conveyor_moving_left = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor.Status.MovingLeft') # Bool
node_conveyor_moving_right = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor.Status.MovingRight') # Bool
node_conveyor_moving_position = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor.Status.Position') # Double
node_conveyor_moving_speed = opcua_client.get_node(f'ns={conveyor_ns_idx};s=Cell.Conveyor.Status.Speed') # Double
# General status nodes:
node_emergency_stop_ok = opcua_client.get_node(f'ns={cell_ns_idx};s=Cell.Status.EStopOK') # Bool
node_operational_stop_ok = opcua_client.get_node(f'ns={cell_ns_idx};s=Cell.Status.OperationalStopOK') # Bool
node_operator_safety_ok = opcua_client.get_node(f'ns={cell_ns_idx};s=Cell.Status.OperatorSafetyOK') # Bool

# Reading and printing values from the nodes:
print('Robot communication OK:', not node_robot_coom_err.read_value())
print('Robot powered ON:', node_robot_powered_on.read_value())
print('Robot busy:', node_robot_busy.read_value())
print('Robot was homed:', node_robot_is_homed.read_value())
if node_robot_position_valid.read_value():
    print('Robot current position:', node_robot_position.read_value())
else:
    print('Robot current position unavailable')

############################################################
# To control the robot, use OPCUA functions - called methods
# First method parameter is always its string identifier created from namespace index and method name
# Folowing parameters are inputs to the function
# It is necesarry to convert the inputs to generalized objects containing the value and a type
# This is done by creating a 'uatype.Variant' object with the value and type as initial parameters
# All available types are part of the 'uatype' module

# !!!
# For the methods to work, the robot has to be in external mode with enabled communication
# Refer to the HMI screen, all preconditions for operation must be met (green indicator)
    # Reintegration - Is fixed by pressing the white REI ACK (Cell 1) keypad button, or the corresponding HMI button
    # Emergency Stop - Is fixed by making sure the E-Stop button is not pressed and then
    #                  pressing the blue SAFETY ACK (Cell 1) keypad button, or the corresponding HMI button
    # Operational Stop - Is fixed by pressing the blue SAFETY ACK (Cell 1) keypad button, or the corresponding HMI button
    # Operator Safety - Is fixed by closing and locking all doors leading to the cell and then
    #                   pressing the blue SAFETY ACK (Cell 1) keypad button, or the corresponding HMI button
    # Robot Communication - Is fixed by switching the robot into EXT mode and presing the 'Reset' HMI button, if necesarry
    # Robot power - Is fixed by pressing the 'Power ON' HMI button, once all other preconditions are met
# !!!
    
# Available methods for robot:
    # SetOverride - to set robot speed override between 0 and 100 %
    # SetFrames - to set robot tool and coordinate frames
    # StartMoveToHome - to return the robot to home, must be done at least once after every robot power-on
    # StartBrakeTest - tests the brakes of the robot, can be done once after turning on the robotic cell, not required
    # StartMoveToPos - move the robot to absolute position in space
    # StartPickPlace - starts a sequence of moves to pick object from moving conveyor and place it into a box, used in main script

# Available methods for gripper:
    # GripperControl to toggle the gripper on and off

# Available methods for conveyor:
    # ConveyorControl - to control the conveyor movement

# !!!
# It is necesarry to check the robot Busy status after issuing a command
# If another command is called while the robot is busy, error will be returned
# !!!

# !!!
# Before any sequence of movements is started after controller boot
# the robot has to be moved into predefined home position
# using the method StartMoveToHome
# !!!

###########################
# SetOverride call example:
# Calling method to set robot speed override to 50 %
# Only argument is unsigned 8 bit integer (called Byte in OPCUA), valid values are between 0 and 100
speed_override = uatype.Variant(100, uatype.Byte)
status, message = node_robot.call_method(f'{robot_ns_idx}:SetOverride', speed_override)
print('SetOverride method call resullt:', message)
# This is the only command which can be called even if the robot is already busy with another command
# It will change the robot speed immediately

#########################
# SetFrames call example:
# First argument is Tool ID (Robot flange has ID of 0) (Vacuum gripper has ID of 1)
# Second argument is Base ID (Robot pedestal has ID of 0) (Conveyor origin has ID of 1)
tool_id = uatype.Variant(1, uatype.Byte)
base_id = uatype.Variant(0, uatype.Byte)
status, message = node_robot.call_method(f'{robot_ns_idx}:SetFrames', tool_id, base_id)
print('SetFrames method call resullt:', message)
print('Waiting for robot to finish command...')
while node_robot_busy.read_value():
    time.sleep(0.2)

###############################
# StartMoveToHome call example:
# The method has no arguments
status, message = node_robot.call_method(f'{robot_ns_idx}:StartMoveToHome')
print('StartMoveToHome method call resullt:', message)
print('Waiting for robot to finish command...')
while node_robot_busy.read_value():
    time.sleep(0.2)

##############################
# StartMoveToPos call example:

# !!!
# StartMoveToPos position is a list containing the pose of the robot in this order:
# X position must be between 470 and 1700 mm - across width of the belt
# Y position must be between -730 and 1400 mm - across length of the belt
# Z position must be between 290 and 1200 mm - height
# A position must be between -270 and 0 degrees - rotation along Z axis, allowed 3/4 of a circle to position the calibration tag
# B position must be between -30 and 30 degrees - rotation along Y axis, default 0, allowed 30 degrees of tilt
# C position must be between 150 and 210 degrees - rotation along X axis, default 180, allowed 30 degrees of tilt
# Tool ID must be 1 for now (Vacuum gripper)
# Base ID must be 0 for now (Robot pedestal)
# !!!

# position = [
#     uatype.Variant(800.0, uatype.Float),
#     uatype.Variant(600.0, uatype.Float),
#     uatype.Variant(500.0, uatype.Float),
#     uatype.Variant(-90.0, uatype.Float),
#     uatype.Variant(0.0, uatype.Float),
#     uatype.Variant(180.0, uatype.Float)
# ]
position = uatype.Variant([
    800.0, # X
    600.0, # Y
    500.0, # Z
    -90.0, # A
      0.0, # B
    180.0, # C
], uatype.Float)
tool_id = uatype.Variant(1, uatype.Byte)
base_id = uatype.Variant(0, uatype.Byte)
status, message = node_robot.call_method(f'{robot_ns_idx}:StartMoveToPos', position, tool_id, base_id)
print('StartMoveToPos method call resullt:', message)
print('Waiting for robot to finish command...')
while node_robot_busy.read_value():
    time.sleep(0.2)

##############################
# GripperControl call example:
# Turn the gripper on
activate = uatype.Variant(True, uatype.Boolean)
status, message = node_gripper.call_method(f'{grippers_ns_idx}:GripperControl', activate)
print('GripperControl method call resullt:', message)

# Wait few seconds
time.sleep(3)

# Turn the gripper off
activate = uatype.Variant(False, uatype.Boolean)
status, message = node_gripper.call_method(f'{grippers_ns_idx}:GripperControl', activate)
print('GripperControl method call resullt:', message)

###############################
# ConveyorControl call example:
# Start moving the conveyor in the left direction
go_left = uatype.Variant(True, uatype.Boolean)
go_right = uatype.Variant(False, uatype.Boolean)
status, message = node_conveyor.call_method(f'{conveyor_ns_idx}:ConveyorControl', go_left, go_right)
print('ConveyorControl method call resullt:', message)

# Wait few seconds
time.sleep(3)

# Start moving the conveyor in the right direction
go_left = uatype.Variant(False, uatype.Boolean)
go_right = uatype.Variant(True, uatype.Boolean)
status, message = node_conveyor.call_method(f'{conveyor_ns_idx}:ConveyorControl', go_left, go_right)
print('ConveyorControl method call resullt:', message)

# Wait few seconds
time.sleep(3)

# Turn the conveyor off
go_left = uatype.Variant(False, uatype.Boolean)
go_right = uatype.Variant(False, uatype.Boolean)
status, message = node_conveyor.call_method(f'{conveyor_ns_idx}:ConveyorControl', go_left, go_right)
print('ConveyorControl method call resullt:', message)

# Disconnect from the server after everything is done
opcua_client.disconnect()
