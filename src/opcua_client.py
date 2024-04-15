# Standard imports
import logging
import time
import multiprocessing
import copy

# External imports
import asyncua.sync as opcua
import asyncua.ua as uatype

# Local imports
from src.logging_setup import setup_logging


# OPCUA Method Status codes
class STATUS():
    SUCCESS = 0
    INTERNAL_ERROR = 1
    INVALID_PARAMETERS = 2
    TIMEOUT = 3
    EMERGENCY_STOP_NOT_OK = 100
    OPERATOR_SAFETY_NOT_OK = 101
    TOTAL_SAFETY_NOT_OK = 102
    ROBOT_BUSY = 1000
    ROBOT_COMMUNICATION_ERROR = 1001
    ROBOT_NOT_POWERED_ON = 1002
    ROBOT_NOT_HOMED = 1003


# Class for passing commands to the command process
class ROBOT_COMMAND():
    EXIT = 10
    ROBOT_SET_OVERRIDE = 100
    ROBOT_SET_FRAMES = 101
    ROBOT_GO_TO_HOME = 102
    ROBOT_START_PICK_PLACE = 103
    CONVEYOR_TOGGLE = 200
    GRIPPER_TOGGLE = 300

    def __init__(self, type, data=None):
        self.type = type
        self.data = data

# local copy of the OPCUA POS6D class, 
# since the OPCUA version cannot be picked
# to be passed across processes using Queue and Manager
class POS6D():
    def __init__(
            self,
            X: float = 0.0,
            Y: float = 0.0,
            Z: float = 0.0,
            A: float = 0.0,
            B: float = 0.0,
            C: float = 0.0,
            Status: int = 0,
            Turn: int = 0,
            ToolFrameID: int = 0,
            BaseFrameID: int = 0,
        ):
        self.X = X
        self.Y = Y
        self.Z = Z
        self.A = A
        self.B = B
        self.C = C
        self.Status = Status
        self.Turn = Turn
        self.ToolFrameID = ToolFrameID
        self.BaseFrameID = BaseFrameID

    def __str__(self):
        return (f'POS6D(' +
            f'X={round(self.X, 2)}, ' +
            f'Y={round(self.Y, 2)}, ' +
            f'Z={round(self.Z, 2)}, ' +
            f'A={round(self.A, 2)}, ' +
            f'B={round(self.B, 2)}, ' +
            f'C={round(self.C, 2)}, ' +
            f'Status={self.Status}, ' +
            f'Turn={self.Turn}, ' +
            f'ToolFrameID={self.ToolFrameID}, ' +
            f'BaseFrameID={self.BaseFrameID})')

    def __repr__(self):
        return (f'POS6D(' +
            f'X={round(self.X, 2)}, ' +
            f'Y={round(self.Y, 2)}, ' +
            f'Z={round(self.Z, 2)}, ' +
            f'A={round(self.A, 2)}, ' +
            f'B={round(self.B, 2)}, ' +
            f'C={round(self.C, 2)}, ' +
            f'Status={self.Status}, ' +
            f'Turn={self.Turn}, ' +
            f'ToolFrameID={self.ToolFrameID}, ' +
            f'BaseFrameID={self.BaseFrameID})')


def status_process_handler(logging_config,
                           opcua_config,
                           shutdown_event,
                           cell_status_dict):
    """
    Process to read values from the PLC OPCUA server.
    Periodically reads robot info from PLC and writes it into 'manag_info_dict', and 'manag_encoder_val.value'
    which is a dictionary read at the same time in the main process.
    """
    # Setup logging
    # Must happen before any logging function call
    log = setup_logging('OPCUA-STATUS', logging_config)

    # Connect server and get nodes
    opcua_client = OPCUA_Client(logging_config, opcua_config)
    opcua_client.connect()
    if not opcua_client.connected:
        return

    # Define list of nodes read by this server
    node_list = [
        opcua_client.node_robot_position,
        opcua_client.node_robot_busy,
        opcua_client.node_conveyor_moving,
        opcua_client.node_conveyor_moving_left,
        opcua_client.node_conveyor_moving_right,
        opcua_client.node_conveyor_position,
        opcua_client.node_conveyor_speed,
        opcua_client.node_robot_gripper_active,
        opcua_client.node_emergency_stop_ok,
        opcua_client.node_operator_safety_ok,
        opcua_client.node_robot_powered_on
    ]

    # Initial disctionary values passed between processes
    cell_status_dict['robot_position'] = POS6D()
    cell_status_dict['robot_busy'] = False
    cell_status_dict['conveyor_moving'] = False
    cell_status_dict['conveyor_moving_left'] = False
    cell_status_dict['conveyor_moving_right'] = False
    cell_status_dict['conveyor_position'] = 0.0
    cell_status_dict['conveyor_speed'] = 0.0
    cell_status_dict['gripper_active'] = False
    cell_status_dict['emergency_stop_ok'] = False
    cell_status_dict['operator_safety_ok'] = False
    cell_status_dict['robot_powered_on'] = False
    cell_status_dict['data_valid'] = False

    # Read values from server forever
    while True:
        try:
            if shutdown_event.is_set():
                log.info(f'OPCUA Status client received EXIT command, disconnecting')
                break

            # Get values from defined nodes
            # Values are ordered in the same way as the nodes
            value_list = opcua_client.read_values(node_list)

            # Assign values from returned list to variables
            cell_status_dict['robot_position'].__dict__ = value_list[0].__dict__
            cell_status_dict['robot_busy'] = value_list[1]
            cell_status_dict['conveyor_moving'] = value_list[2]
            cell_status_dict['conveyor_moving_left'] = value_list[3]
            cell_status_dict['conveyor_moving_right'] = value_list[4]
            cell_status_dict['conveyor_position'] = round(value_list[5], 2)
            cell_status_dict['conveyor_speed'] = round(value_list[6], 2)
            cell_status_dict['gripper_active'] = value_list[7]
            cell_status_dict['emergency_stop_ok'] = value_list[8]
            cell_status_dict['operator_safety_ok'] = value_list[9]
            cell_status_dict['robot_powered_on'] = value_list[10]
            cell_status_dict['data_valid'] = True

        except Exception as e:
            log.error(f'OPCUA Status client disconnected: {e}')
            break

    cell_status_dict['data_valid'] = False
    shutdown_event.set()
    opcua_client.disconnect()


def command_process_handler(logging_config,
                            opcua_config,
                            shutdown_event,
                            abort_commands_event,
                            cell_command_queue,
                            cell_status_dict):
    """
    Process to control the cell using OPCUA methods
    """
    # Setup logging
    # Must happen before any logging function call
    log = setup_logging('OPCUA-COMMAND', logging_config)

    # Connect server and get nodes
    opcua_client = OPCUA_Client(logging_config, opcua_config)
    opcua_client.connect()
    if not opcua_client.connected:
        return

    while not cell_status_dict['data_valid']:
        pass

    while True:
        try:
            if shutdown_event.is_set():
                log.info(f'OPCUA Command client received EXIT command, disconnecting')
                break

            if abort_commands_event.is_set():
                while not cell_command_queue.empty():
                    cell_command_queue.get()
                abort_commands_event.clear()

            command = cell_command_queue.get()
            assert isinstance(command, ROBOT_COMMAND)

            command_type = command.type
            command_data = command.data

            if command_type == ROBOT_COMMAND.ROBOT_SET_OVERRIDE:
                log.info(f'Set Override method call')
                while True:
                    result, message = opcua_client.set_robot_speed_override(command_data)
                    if result == STATUS.SUCCESS or shutdown_event.is_set() or abort_commands_event.is_set():
                        break
                    time.sleep(0.2)

            elif command_type == ROBOT_COMMAND.ROBOT_SET_FRAMES:
                log.info(f'Set Frames method call')
                while True:
                    result, message = opcua_client.set_robot_frame(command_data[0], command_data[1])
                    if result == STATUS.SUCCESS or shutdown_event.is_set() or abort_commands_event.is_set():
                        break
                    time.sleep(0.2)

            elif command_type == ROBOT_COMMAND.ROBOT_GO_TO_HOME:
                log.info(f'Move To Home method call')
                while True:
                    result, message = opcua_client.start_move_to_home()
                    if result == STATUS.SUCCESS or shutdown_event.is_set() or abort_commands_event.is_set():
                        break
                    time.sleep(0.2)

            elif command_type == ROBOT_COMMAND.ROBOT_START_PICK_PLACE:
                log.debug(f'Pick Place method call')
                max_pick_distance = command_data[0]
                conveyor_init_pos = command_data[1]

                conveyor_tigger_pos = command_data[2]
                start_pos = command_data[3]
                pre_pick_pos = command_data[4]
                pick_pos = command_data[5]
                post_pick_pos = command_data[6]
                place_pos = command_data[7]

                new_conveyor_tigger_pos = copy.deepcopy(command_data[2])
                new_start_pos = copy.deepcopy(command_data[3])
                new_pre_pick_pos = copy.deepcopy(command_data[4])
                new_pick_pos = copy.deepcopy(command_data[5])
                new_post_pick_pos = copy.deepcopy(command_data[6])
                new_place_pos = copy.deepcopy(command_data[7])

                while True:
                    # Compute difference between conveyor position when the data was assembled
                    # and current conveyor position
                    conveyor_diff = cell_status_dict['conveyor_position'] - conveyor_init_pos

                    # Use the computed difference to offset the position values
                    new_conveyor_tigger_pos = conveyor_tigger_pos + conveyor_diff

                    new_start_pos = start_pos

                    new_pre_pick_pos.X = pre_pick_pos.X + conveyor_diff
                    new_pick_pos.X = pick_pos.X + conveyor_diff
                    new_post_pick_pos.X = post_pick_pos.X + conveyor_diff

                    new_place_pos = place_pos

                    log.debug('------------PICK-PLACE--------------')
                    log.debug(f'Conveyor difference: {conveyor_diff}')
                    log.debug(f'Conveyor trigger position: {conveyor_tigger_pos}')
                    log.debug(f'Start position: {start_pos}')
                    log.debug(f'Pre-Pick position: {pre_pick_pos}')
                    log.debug(f'Pick position: {pick_pos}')
                    log.debug(f'Post-Pick position: {post_pick_pos}')
                    log.debug(f'Place position: {place_pos}')
                    log.debug('------------------------------------')

                    # Check if the pick position does not excees the bounds of the picking area
                    if new_post_pick_pos.X > max_pick_distance:
                        log.warning(f'Missed item')
                        break

                    log.debug(f'Pick Place attempt with conveyor difference from initial value: {round(conveyor_diff, 2)}')
                    result, message = opcua_client.start_pick_place(
                        new_conveyor_tigger_pos,
                        new_start_pos,
                        new_pre_pick_pos,
                        new_pick_pos,
                        new_post_pick_pos,
                        new_place_pos,
                    )

                    if result == STATUS.SUCCESS or shutdown_event.is_set() or abort_commands_event.is_set():
                        break

                    time.sleep(0.2)

            elif command_type == ROBOT_COMMAND.CONVEYOR_TOGGLE:
                opcua_client.conveyor_control(command_data[0], command_data[1])

            elif command_type == ROBOT_COMMAND.GRIPPER_TOGGLE:
                opcua_client.gripper_control(command_data)

            else:
                log.warning(f'Wrong command sent to control server: {command_type}')

        except Exception as e:
            log.error(f'OPCUA Command client disconnected: {e}')
            break
    
    shutdown_event.set()
    opcua_client.disconnect()


# Load OPCUA data type definitions in the main process
# Types loaded in separate processes are not accesible in the calling program
def load_opcua_datatypes(logging_config, opcua_config):
    assert isinstance(opcua_config.ip, str)
    assert isinstance(opcua_config.port, int)
    assert isinstance(opcua_config.username, str)
    assert isinstance(opcua_config.password, str)

    ip = opcua_config.ip
    port = opcua_config.port
    username = opcua_config.username
    password = opcua_config.password

    # Setup logging
    # Must happen before any logging function call
    log = setup_logging('OPCUA', logging_config)

    if username and password:
        client = opcua.Client(f'opc.tcp://{username}:{password}@{ip}:{port}/')
    else:
        client = opcua.Client(f'opc.tcp://{ip}:{port}/')

    try:
        client.connect()
        client.load_data_type_definitions()
        client.disconnect()
        log.info(f'Loaded OPCUA datatypes from server at {ip}:{port}')
    except Exception as e:
        log.error(f'Filed to load OPCUA datatypes from server at {ip}:{port}: {e}')


class OPCUA_Client:
    """
    Class for OPCUA communication with the PLC.
    """

    def __init__(self, logging_config, opcua_config):
        """
        Constructor
        """
        assert isinstance(opcua_config.ip, str)
        assert isinstance(opcua_config.port, int)
        assert isinstance(opcua_config.username, str)
        assert isinstance(opcua_config.password, str)
        assert isinstance(opcua_config.workplace_namespace_uri, str)
        assert isinstance(opcua_config.robots_namespace_uri, str)
        assert isinstance(opcua_config.transport_namespace_uri, str)

        # Setup logging
        # Must happen before any logging function call
        self.log = setup_logging('OPCUA', logging_config)
        logging.getLogger().setLevel(logging.CRITICAL)

        self.ip = opcua_config.ip
        self.port = opcua_config.port
        self.username = opcua_config.username
        self.password = opcua_config.password
        self.workplace_namespace_uri = opcua_config.workplace_namespace_uri
        self.robots_namespace_uri = opcua_config.robots_namespace_uri
        self.transport_namespace_uri = opcua_config.transport_namespace_uri

        self.client = None
        self.connected = False
        self.timeout = 4  # Every request expects answer in this time (in seconds)
        self.secure_channel_timeout = 300_000  # Timeout for the secure channel (in milliseconds), it should be equal to the timeout set on the PLC
        self.session_timeout = 30_000  # Timeout for the session (in milliseconds), it should be equal to the timeout set on the PLC

    def connect(self):
        """
        Connects to the server
        """

        if self.username and self.password:
            self.client = opcua.Client(f'opc.tcp://{self.username}:{self.password}@{self.ip}:{self.port}/', self.timeout)
        else:
            self.client = opcua.Client(f'opc.tcp://{self.ip}:{self.port}/', self.timeout)
        self.client.secure_channel_timeout = self.secure_channel_timeout
        self.client.session_timeout = self.session_timeout

        try:
            self.client.connect()
            self.client.load_data_type_definitions()
            self.get_nodes()
            self.connected = True
            self.log.info(f'OPCUA client connected to server at {self.ip}:{self.port}')
        except Exception as e:
            self.connected = False
            self.log.critical(f'OPCUA client failed to connect to server at {self.ip}:{self.port}: {e}')

    def disconnect(self):
        """
        Disconnects from the server
        """

        self.client.disconnect()
        self.connected = False
        self.log.info(f'OPCUA client disconnected from {self.ip}:{self.port}')

    def get_nodes(self):
        """
        Using the client.get_node() method, get all requied nodes from the PLC server.
        All nodes are then passed to the client.register_nodes() method, which notifies the server
        that it should perform optimizations for reading / writing operations for these nodes.
        """

        self.robot_ns_idx = self.client.get_namespace_index(self.robots_namespace_uri)
        self.node_robot = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32')
        self.node_robot_busy = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Busy')
        self.node_robot_position = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Position')
        self.node_robot_position_valid = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.PositionValid')
        self.node_robot_speed_override = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.SpeedOverride')
        self.node_robot_speed_override_valid = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.SpeedOverrideValid')
        self.node_robot_powered_on = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.PoweredON')
        self.node_robot_gripper_active = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.GripperActive')

        self.transport_ns_idx = self.client.get_namespace_index(self.transport_namespace_uri)
        self.node_conveyor = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor')
        self.node_conveyor_moving = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor.Moving')
        self.node_conveyor_moving_left = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor.MovingLeft')
        self.node_conveyor_moving_right = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor.MovingRight')
        self.node_conveyor_position = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor.Position')
        self.node_conveyor_speed = self.client.get_node(f'ns={self.transport_ns_idx};s=Cell.Conveyor.Speed')

        self.workplace_ns_idx = self.client.get_namespace_index(self.workplace_namespace_uri)
        self.node_emergency_stop_ok = self.client.get_node(f'ns={self.workplace_ns_idx};s=Cell.EStopOK')
        self.node_operator_safety_ok = self.client.get_node(f'ns={self.workplace_ns_idx};s=Cell.OperatorSafetyOK')

        # Register all nodes for faster read / write access.
        # The client.register_nodes() only takes a list of nodes as input, and returns list of
        # registered nodes as output, so single node is wrapped in a list and then received
        # as first element of a list.
        for _, value in self.__dict__.items():
            if type(value) == opcua.SyncNode:
                value = self.client.register_nodes([value])[0]

    def read_values(self, node_list):
        """
        Read several values from the server at once
        """
        return self.client.read_values(node_list)
    
    def wait_for_robot(self):
        self.log.info(f'Waiting for robot to finish operation')
        while self.node_robot_busy.read_value() or not self.node_robot_powered_on.read_value():
            time.sleep(0.1)

    def wait_for_operator_safety(self):
        self.log.info(f'Waiting for cell operator safety to be acknowledged')
        while not self.node_operator_safety_ok.read_value():
            time.sleep(0.1)

    def set_robot_speed_override(self, speed_override: int):
        assert isinstance(speed_override, int)

        result, message = self.node_robot.call_method(
            f'{self.robot_ns_idx}:SetOverride',
            uatype.Variant(speed_override, uatype.Byte)
        )
        if result == STATUS.SUCCESS:
            self.log.info(f'Set robot speed override to {speed_override} %')
        else:
            self.log.error(f'Failed to set robot speed override: {message}')
        return result, message
    
    def set_robot_frame(self, tool_id: int, base_id: int):
        assert isinstance(tool_id, int)
        assert isinstance(base_id, int)

        result, message = self.node_robot.call_method(
            f'{self.robot_ns_idx}:SetFrames',
            uatype.Variant(tool_id, uatype.Byte),
            uatype.Variant(base_id, uatype.Byte)
        )
        if result == STATUS.SUCCESS:
            self.log.info(f'Set robot Tool ID to {tool_id} and Base ID to {base_id}')
        elif result == STATUS.ROBOT_BUSY:
            pass
        else:
            self.log.error(f'Failed to set robot Tool and Base IDs: {message}')
        return result, message
    
    def start_move_to_home(self):
        result, message = self.node_robot.call_method(
            f'{self.robot_ns_idx}:StartMoveToHome'
        )
        if result == STATUS.SUCCESS:
            self.log.info(f'Sent robot to home position')
        elif result == STATUS.ROBOT_BUSY:
            pass
        else:
            self.log.error(f'Failed to send robot to home position: {message}')
        return result, message
        
    def start_pick_place(self,
                         conveyor_tigger_pos, 
                         start_pos, 
                         pre_pick_pos, 
                         pick_pos, 
                         post_pick_pos, 
                         place_pos):
        assert isinstance(conveyor_tigger_pos, (int, float))
        assert isinstance(start_pos, POS6D)
        assert isinstance(pre_pick_pos, POS6D)
        assert isinstance(pick_pos, POS6D)
        assert isinstance(post_pick_pos, POS6D)
        assert isinstance(place_pos, POS6D)

        ua_conveyor_tigger_pos = uatype.Variant(conveyor_tigger_pos, uatype.Double)
        ua_start_pos = uatype.POS6D()
        ua_start_pos.__dict__ = start_pos.__dict__
        ua_pre_pick_pos = uatype.POS6D()
        ua_pre_pick_pos.__dict__ = pre_pick_pos.__dict__
        ua_pick_pos = uatype.POS6D()
        ua_pick_pos.__dict__ = pick_pos.__dict__
        ua_post_pick_pos = uatype.POS6D()
        ua_post_pick_pos.__dict__ = post_pick_pos.__dict__
        ua_place_pos = uatype.POS6D()
        ua_place_pos.__dict__ = place_pos.__dict__

        result, message = self.node_robot.call_method(
            f'{self.robot_ns_idx}:StartPickPlace',
            ua_conveyor_tigger_pos,
            ua_start_pos,
            ua_pre_pick_pos,
            ua_pick_pos,
            ua_post_pick_pos,
            ua_place_pos,
        )
        if result == STATUS.SUCCESS:
            self.log.info(f'Started Pick & Place program')
        elif result == STATUS.ROBOT_BUSY:
            pass
        else:
            self.log.error(f'Failed to start Pick & Place program: {message}')
        return result, message

    def conveyor_control(self, move_left: bool, move_right: bool):
        assert isinstance(move_left, bool)
        assert isinstance(move_right, bool)

        status, message = self.node_conveyor.call_method(
            f'{self.transport_ns_idx}:ConveyorControl',
            uatype.Variant(move_left, uatype.Boolean),
            uatype.Variant(move_right, uatype.Boolean),
        )
        if status == STATUS.SUCCESS:
            if not move_left and not move_right:
                self.log.info(f'Stopped conveyor')
            elif move_left:
                self.log.info(f'Started conveyor in the left direction')
            elif move_right:
                self.log.info(f'Started conveyor in the right direction')
        else:
            self.log.error(f'Failed to start conveyor: {message}')
        return status, message
    
    def gripper_control(self, grip: bool):
        assert isinstance(grip, bool)

        status, message = self.node_robot.call_method(
            f'{self.robot_ns_idx}:GripperControl', 
            uatype.Variant(grip, uatype.Boolean)
        )
        if status == STATUS.SUCCESS:
            if grip:
                self.log.info(f'Enabled gripper')
            else:
                self.log.info(f'Disabled gripper')
        else:
            self.log.error(f'Failed to enable gripper: {message}')
        return status, message
