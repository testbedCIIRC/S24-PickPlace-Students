# Standard imports
import logging
import time
import multiprocessing

# External imports
import asyncua.sync as opcua
import asyncua.ua as uatype


# OPCUA Method Status codes
class STATUS():
    SUCCESS = 0
    ROBOT_NOT_HOMED = 2000
    ROBOT_BUSY = 2000
    EMERGENCY_STOP_NOT_OK = 2000
    OPERATOR_SAFETY_NOT_OK = 2000


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


def status_process_handler(logging_config,
                           config,
                           multiprocess_dict):
    """
    Process to read values from PLC server.
    Periodically reads robot info from PLC and writes it into 'manag_info_dict', and 'manag_encoder_val.value'
    which is a dictionary read at the same time in the main process.
    """

    # Connect server and get nodes
    opcua_client = OPCUA_Client(logging_config, config)
    opcua_client.connect()
    if not opcua_client.connected:
        return
    
    opcua_client.get_nodes()

    # Define list of nodes read by this server
    node_list = [
        opcua_client.node_robot_position,
        opcua_client.node_robot_busy,
        opcua_client.node_conveyor_moving,
        opcua_client.node_conveyor_moving_left,
        opcua_client.node_conveyor_moving_right,
        opcua_client.node_conveyor_position,
        opcua_client.node_conveyor_speed,
        opcua_client.node_gripper_active,
        opcua_client.node_emergency_stop_ok,
        opcua_client.node_operator_safety_ok
    ]

    # Initial disctionary values passed between processes
    robot_position = {'X': 0.0,
                      'Y': 0.0,
                      'Z': 0.0,
                      'A': 0.0,
                      'B': 0.0,
                      'C': 0.0,
                      'Status': 0,
                      'Turn': 0,
                      'ToolFrameID': 0,
                      'BaseFrameID': 0}
    multiprocess_dict['robot_position'] = robot_position
    multiprocess_dict['robot_busy'] = False
    multiprocess_dict['conveyor_moving'] = False
    multiprocess_dict['conveyor_moving_left'] = False
    multiprocess_dict['conveyor_moving_right'] = False
    multiprocess_dict['conveyor_position'] = 0.0
    multiprocess_dict['conveyor_speed'] = 0.0
    multiprocess_dict['gripper_active'] = False
    multiprocess_dict['emergency_stop_ok'] = False
    multiprocess_dict['operator_safety_ok'] = False
    multiprocess_dict['data_valid'] = False
    multiprocess_dict['exit'] = False

    # Read values from server forever
    while True:
        try:
            if multiprocess_dict['exit']:
                opcua_client.log.info(f'OPCUA Status client received EXIT command, disconnecting')
                break

            # Get values from defined nodes
            # Values are ordered in the same way as the nodes
            value_list = opcua_client.read_values(node_list)

            # Assign values from returned list to variables
            robot_position['X'] = value_list[0].X
            robot_position['Y'] = value_list[0].Y
            robot_position['Z'] = value_list[0].Z
            robot_position['A'] = value_list[0].A
            robot_position['B'] = value_list[0].B
            robot_position['C'] = value_list[0].C
            robot_position['Status'] = value_list[0].Status
            robot_position['Turn'] = value_list[0].Turn
            robot_position['ToolFrameID'] = value_list[0].ToolFrameID
            robot_position['BaseFrameID'] = value_list[0].BaseFrameID
            multiprocess_dict['robot_position'] = robot_position
            multiprocess_dict['robot_busy'] = value_list[1]
            multiprocess_dict['conveyor_moving'] = value_list[2]
            multiprocess_dict['conveyor_moving_left'] = value_list[3]
            multiprocess_dict['conveyor_moving_right'] = value_list[4]
            multiprocess_dict['conveyor_position'] = round(value_list[5], 2)
            multiprocess_dict['conveyor_speed'] = round(value_list[6], 2)
            multiprocess_dict['gripper_active'] = value_list[7]
            multiprocess_dict['emergency_stop_ok'] = value_list[8]
            multiprocess_dict['operator_safety_ok'] = value_list[9]
            multiprocess_dict['data_valid'] = True

        except Exception as e:
            opcua_client.log.error(f'OPCUA Status client disconnected: {e}')
            break

    multiprocess_dict['data_valid'] = False
    opcua_client.disconnect()


def command_process_handler(logging_config,
                            config,
                            multiprocess_queue: multiprocessing.Queue):
    """
    Process to control teh cell using OPCUA methods
    """
    # Connect server and get nodes
    opcua_client = OPCUA_Client(logging_config, config)
    opcua_client.connect()
    if not opcua_client.connected:
        return

    opcua_client.get_nodes()

    while True:
        try:
            command = multiprocess_queue.get()
            assert isinstance(command, ROBOT_COMMAND)

            command_type = command.type
            command_data = command.data

            if command_type == ROBOT_COMMAND.ROBOT_SET_OVERRIDE:
                opcua_client.set_robot_speed_override(command_data)

            elif command_type == ROBOT_COMMAND.ROBOT_SET_FRAMES:
                opcua_client.set_robot_frame(command_data[0], command_data[1])

            elif command_type == ROBOT_COMMAND.ROBOT_GO_TO_HOME:
                opcua_client.start_move_to_home()

            elif command_type == ROBOT_COMMAND.ROBOT_START_PICK_PLACE:
                opcua_client.start_pick_place(command_data[0], 
                                              command_data[1], 
                                              command_data[2], 
                                              command_data[3], 
                                              command_data[4], 
                                              command_data[5], 
                                              command_data[6], 
                                              command_data[7])

            elif command_type == ROBOT_COMMAND.CONVEYOR_TOGGLE:
                opcua_client.conveyor_control(command_data[0], command_data[1])

            elif command_type == ROBOT_COMMAND.GRIPPER_TOGGLE:
                opcua_client.gripper_control(command_data)

            elif command_type == ROBOT_COMMAND.EXIT:
                opcua_client.log.info(f'OPCUA Command client received EXIT command, disconnecting')
                break

            else:
                opcua_client.log.warning(f'Wrong command sent to control server: {command_type}')

        except Exception as e:
            opcua_client.log.error(f'OPCUA Command client disconnected: {e}')
            break
    
    opcua_client.disconnect()


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
        assert isinstance(opcua_config.cell_namespace_uri, str)
        assert isinstance(opcua_config.robot_namespace_uri, str)
        assert isinstance(opcua_config.conveyor_namespace_uri, str)
        assert isinstance(opcua_config.grippers_namespace_uri, str)

        self.ip = opcua_config.ip
        self.port = opcua_config.port
        self.username = opcua_config.username
        self.password = opcua_config.password
        self.cell_namespace_uri = opcua_config.cell_namespace_uri
        self.robot_namespace_uri = opcua_config.robot_namespace_uri
        self.conveyor_namespace_uri = opcua_config.conveyor_namespace_uri
        self.grippers_namespace_uri = opcua_config.grippers_namespace_uri

        self.client = None
        self.connected = False
        self.timeout = 4  # Every request expects answer in this time (in seconds)
        self.secure_channel_timeout = 300_000  # Timeout for the secure channel (in milliseconds), it should be equal to the timeout set on the PLC
        self.session_timeout = 30_000  # Timeout for the session (in milliseconds), it should be equal to the timeout set on the PLC

        # Setup logging
        logging.root.setLevel(logging.CRITICAL)
        self.log = logging.getLogger('PickPlace-Logger')
        hdl = logging.StreamHandler()
        if logging_config.level == 'DEBUG':
            self.log.setLevel(logging.DEBUG)
            hdl.setLevel(logging.DEBUG)
        elif logging_config.level == 'INFO':
            self.log.setLevel(logging.INFO)
            hdl.setLevel(logging.INFO)
        elif logging_config.level == 'WARNING':
            self.log.setLevel(logging.WARNING)
            hdl.setLevel(logging.WARNING)
        elif logging_config.level == 'ERROR':
            self.log.setLevel(logging.ERROR)
            hdl.setLevel(logging.ERROR)
        elif logging_config.level == 'CRITICAL':
            self.log.setLevel(logging.CRITICAL)
            hdl.setLevel(logging.CRITICAL)
        else:
            self.log.setLevel(logging.WARNING)
            hdl.setLevel(logging.WARNING)
        log_formatter = logging.Formatter(fmt='[%(asctime)s] [%(levelname)s] - %(message)s',
                                          datefmt='%Y-%m-%d %H:%M:%S')
        hdl.setFormatter(log_formatter)
        self.log.addHandler(hdl)

    def connect(self):
        """
        Connects to the server
        """

        if self.username and self.password:
            self.client = opcua.Client(f'opc.tcp://{self.username}:{self.password}@{self.ip}:{self.port}/', self.timeout)
            self.log.info(f'Initialized OPCUA client as user {self.username}')
        else:
            self.client = opcua.Client(f'opc.tcp://{self.ip}:{self.port}/', self.timeout)
            self.log.info(f'Initialized OPCUA client as anonymous user')
        self.client.secure_channel_timeout = self.secure_channel_timeout
        self.client.session_timeout = self.session_timeout

        try:
            self.client.connect()
            self.client.load_data_type_definitions()
            self.connected = True
            self.log.info(f'OPCUA client connected to server at {self.ip}')
        except Exception as e:
            self.connected = False
            self.log.error(f'OPCUA client failed to connect to server at {self.ip}: {e}')

    def disconnect(self):
        """
        Disconnects from the server
        """

        self.client.disconnect()
        self.connected = False

    def get_nodes(self):
        """
        Using the client.get_node() method, get all requied nodes from the PLC server.
        All nodes are then passed to the client.register_nodes() method, which notifies the server
        that it should perform optimizations for reading / writing operations for these nodes.
        """

        self.robot_ns_idx = self.client.get_namespace_index(self.robot_namespace_uri)
        self.node_robot = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32')
        self.node_robot_busy = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Status.Busy')
        self.node_robot_position = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Status.Position')
        self.node_robot_position_valid = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Status.PositionValid')
        self.node_robot_speed_override = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Status.SpeedOverride')
        self.node_robot_speed_override_valid = self.client.get_node(f'ns={self.robot_ns_idx};s=Cell.Robots.R32.Status.SpeedOverrideValid')

        self.conveyor_ns_idx = self.client.get_namespace_index(self.conveyor_namespace_uri)
        self.node_conveyor = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor')
        self.node_conveyor_moving = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor.Status.Moving')
        self.node_conveyor_moving_left = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor.Status.MovingLeft')
        self.node_conveyor_moving_right = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor.Status.MovingRight')
        self.node_conveyor_position = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor.Status.Position')
        self.node_conveyor_speed = self.client.get_node(f'ns={self.conveyor_ns_idx};s=Cell.Conveyor.Status.Speed')

        self.grippers_ns_idx = self.client.get_namespace_index(self.grippers_namespace_uri)
        self.node_gripper = self.client.get_node(f'ns={self.grippers_ns_idx};s=Cell.Grippers.G1')
        self.node_gripper_active = self.client.get_node(f'ns={self.grippers_ns_idx};s=Cell.Grippers.G1.Status.Active')

        self.cell_ns_idx = self.client.get_namespace_index(self.cell_namespace_uri)
        self.node_emergency_stop_ok = self.client.get_node(f'ns={self.cell_ns_idx};s=Cell.Status.EStopOK')
        self.node_operator_safety_ok = self.client.get_node(f'ns={self.cell_ns_idx};s=Cell.Status.OperatorSafetyOK')

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
        while self.node_robot_busy.read_value():
            time.sleep(0.1)

    def wait_for_operator_safety(self):
        self.log.info(f'Waiting for cell operator safety to be acknowledged')
        while not self.node_operator_safety_ok.read_value():
            time.sleep(0.1)

    def set_robot_speed_override(self, speed_override: int):
        assert isinstance(speed_override, int)

        result, message = self.node_robot.call_method(f'{self.robot_ns_idx}:SetOverride',
                                                      uatype.Variant(speed_override, uatype.Byte))
        if result == STATUS.SUCCESS:
            self.log.info(f'Set robot speed override to {speed_override} %')
        else:
            self.log.error(f'Failed to set robot speed override: {message}')
        return result, message
    
    def set_robot_frame(self, tool_id: int, base_id: int):
        assert isinstance(tool_id, int)
        assert isinstance(base_id, int)

        while True:
            result, message = self.node_robot.call_method(f'{self.robot_ns_idx}:SetFrames',
                                                        uatype.Variant(tool_id, uatype.Byte),
                                                        uatype.Variant(base_id, uatype.Byte))
            if result == STATUS.SUCCESS:
                self.log.info(f'Set robot Tool ID to {tool_id} and Base ID to {base_id}')
                break
            elif result == STATUS.ROBOT_BUSY:
                self.wait_for_robot()
            elif result == STATUS.OPERATOR_SAFETY_NOT_OK:
                #self.wait_for_operator_safety()
                self.log.error(f'Failed to start Pick & Place program: {message}')
                break
            else:
                self.log.error(f'Failed to set robot Tool and Base IDs: {message}')
                break
        return result, message
    
    def start_move_to_home(self):
        self.wait_for_robot()
        while True:
            result, message = self.node_robot.call_method(f'{self.robot_ns_idx}:StartMoveToHome')
            if result == STATUS.SUCCESS:
                self.log.info(f'Sent robot to home position')
                break
            elif result == STATUS.ROBOT_BUSY:
                self.wait_for_robot()
            elif result == STATUS.OPERATOR_SAFETY_NOT_OK:
                #self.wait_for_operator_safety()
                self.log.error(f'Failed to start Pick & Place program: {message}')
                break
            else:
                self.log.error(f'Failed to send robot to home position: {message}')
                break
            time.sleep(0.2)
        return result, message
        
    def start_pick_place(self, 
                         conveyor_tigger_pos, 
                         start_pos, 
                         pre_pick_pos, 
                         pick_pos, 
                         post_pick_pos, 
                         place_pos, 
                         tool_id, 
                         base_id):
        assert isinstance(conveyor_tigger_pos, (int, float))
        assert isinstance(start_pos, list)
        assert len(start_pos) == 8
        for coord in start_pos:
            assert isinstance(coord, (int, float))
        assert isinstance(pre_pick_pos, list)
        assert len(pre_pick_pos) == 6
        for coord in pre_pick_pos:
            assert isinstance(coord, (int, float))
        assert isinstance(pick_pos, list)
        assert len(pick_pos) == 6
        for coord in pick_pos:
            assert isinstance(coord, (int, float))
        assert isinstance(post_pick_pos, list)
        assert len(post_pick_pos) == 6
        for coord in post_pick_pos:
            assert isinstance(coord, (int, float))
        assert isinstance(place_pos, list)
        assert len(place_pos) == 8
        for coord in place_pos:
            assert isinstance(coord, (int, float))
        assert isinstance(tool_id, int)
        assert isinstance(base_id, int)

        self.wait_for_robot()
        while True:
            result, message = self.node_robot.call_method(f'{self.robot_ns_idx}:StartPickPlace',
                                                          uatype.Variant(conveyor_tigger_pos, uatype.Double),
                                                          uatype.Variant(start_pos[0:6], uatype.Float), # Coords
                                                          uatype.Variant(start_pos[6:8], uatype.Byte), # Configuration
                                                          uatype.Variant(pre_pick_pos, uatype.Float),
                                                          uatype.Variant(pick_pos, uatype.Float),
                                                          uatype.Variant(post_pick_pos, uatype.Float),
                                                          uatype.Variant(place_pos[0:6], uatype.Float), # Coords
                                                          uatype.Variant(place_pos[6:8], uatype.Byte), # Configuration
                                                          uatype.Variant(tool_id, uatype.Byte),
                                                          uatype.Variant(base_id, uatype.Byte))
            if result == STATUS.SUCCESS:
                self.log.info(f'Started Pick & Place program')
                break
            elif result == STATUS.ROBOT_BUSY:
                self.wait_for_robot()
            elif result == STATUS.OPERATOR_SAFETY_NOT_OK:
                #self.wait_for_operator_safety()
                self.log.error(f'Failed to start Pick & Place program: {message}')
                break
            else:
                self.log.error(f'Failed to start Pick & Place program: {message}')
                break
            time.sleep(0.2)
        return result, message

    def conveyor_control(self, move_left: bool, move_right: bool):
        assert isinstance(move_left, bool)
        assert isinstance(move_right, bool)

        status, message = self.node_conveyor.call_method(f'{self.conveyor_ns_idx}:ConveyorControl', 
                                                         uatype.Variant(move_left, uatype.Boolean), 
                                                         uatype.Variant(move_right, uatype.Boolean))
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

        status, message = self.node_gripper.call_method(f'{self.grippers_ns_idx}:GripperControl', 
                                                         uatype.Variant(grip, uatype.Boolean))
        if status == STATUS.SUCCESS:
            if grip:
                self.log.info(f'Enabled gripper')
            else:
                self.log.info(f'Disabled gripper')
        else:
            self.log.error(f'Failed to enable gripper: {message}')
        return status, message