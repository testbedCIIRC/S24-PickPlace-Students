from json import tool
import asyncua.sync as opcua
import asyncua.ua as uatype
import logging
import time

PLC_OPCUA_URL = 'opc.tcp://10.100.0.120:4840/'

workplace_ns_uri = "http://w4.ti40.cz"
robots_ns_uri = "http://robots.ti40.cz"
transport_ns_uri = "http://transportation.ti40.cz"

class RobotControl():
    def connect(self):
        self.opcua_client = opcua.Client(PLC_OPCUA_URL)
        self.opcua_client.connect()
        
        self.types = self.opcua_client.load_data_type_definitions()
        self.workplace_ns_idx = self.opcua_client.get_namespace_index(workplace_ns_uri)
        self.robots_ns_idx = self.opcua_client.get_namespace_index(robots_ns_uri)
        self.transport_ns_idx = self.opcua_client.get_namespace_index(transport_ns_uri)

        self.node_robot = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32')
        self.node_robot_coom_err = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.CommunicationError') # Bool
        self.node_robot_powered_on = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.PoweredON') # Bool
        self.node_robot_busy = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.Busy') # Bool
        self.node_robot_is_homed = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.IsHomed') # Bool
        self.node_robot_position = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.Position') # POS6D
        self.node_robot_position_valid = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.PositionValid') # Bool
        self.node_robot_gripper_active = self.opcua_client.get_node(f'ns={ self.robots_ns_idx};s=Cell.Robots.R32.GripperActive') # Bool
        # Conveyor nodes:
        self.node_conveyor = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor')
        self.node_conveyor_moving = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor.Moving') # Bool
        self.node_conveyor_moving_left = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor.MovingLeft') # Bool
        self.node_conveyor_moving_right = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor.MovingRight') # Bool
        self.node_conveyor_moving_position = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor.Position') # Double
        self.node_conveyor_moving_speed = self.opcua_client.get_node(f'ns={ self.transport_ns_idx};s=Cell.Conveyor.Speed') # Double
        # General status nodes:
        self.node_emergency_stop_ok = self.opcua_client.get_node(f'ns={self.workplace_ns_idx};s=Cell.EStopOK') # Bool
        self.node_operator_safety_ok = self.opcua_client.get_node(f'ns={self.workplace_ns_idx};s=Cell.OperatorSafetyOK') # Bool
        
        self.initialized = True

        print('Robot communication OK:', not self.node_robot_coom_err.read_value())
        print('Robot powered ON:', self.node_robot_powered_on.read_value())
        print('Robot busy:', self.node_robot_busy.read_value())
        print('Robot was homed:', self.node_robot_is_homed.read_value())
        print('Robot is ready:', self.is_ready())

    def get_position(self):
        return self.node_robot_position.read_value()
    
    def setFrames(self, tool_id=0, base_id=0):
        if not self.initialized:
            return False
        self.tool_id_int = tool_id
        self.base_id_int = base_id

        self.tool_id = uatype.Variant(tool_id, uatype.Byte)
        self.base_id = uatype.Variant(base_id, uatype.Byte)
        status, message = self.node_robot.call_method(f'{self.robots_ns_idx}:SetFrames', self.tool_id, self.base_id)
        print('SetFrames method call resullt:', message)
        print('Waiting for robot to finish command...')
        while self.node_robot_busy.read_value():
            time.sleep(0.2)
        return True

    def set_speed(self, speed=10):
        speed_override = uatype.Variant(speed, uatype.Byte)
        status, message = self.node_robot.call_method(f'{self.robots_ns_idx}:SetOverride', speed_override)
        print('SetOverride method call resullt:', message)
    def go_home(self):
        if not self.is_ready():
            return False
        status, message = self.node_robot.call_method(f'{self.robots_ns_idx}:StartMoveToHome')
        print('StartMoveToHome method call resullt:', message)
        print('Waiting for robot to finish command...')
        while self.node_robot_busy.read_value():
            time.sleep(0.2)
        return True

    def XYZABC_2_position(self, X,Y,Z,A,B,C):
        position = uatype.POS6D()
        position.X = X
        position.Y = Y
        position.Z = Z
        position.A = A
        position.B = B
        position.C = C
        position.Status = 0
        position.Turn = 0
        position.ToolFrameID = self.tool_id_int
        position.BaseFrameID = self.base_id_int
        return position

    def go_position(self, position):
        if not self.is_ready():
            return False
        status, message = self.node_robot.call_method(f'{self.robots_ns_idx}:StartMoveToPos', position)
        print('StartMoveToPos method call resullt:', message)
        print('Waiting for robot to finish command...')
        while self.node_robot_busy.read_value():
            time.sleep(0.2)
        return True

    def is_ready(self):
        ready = True
        if not self.initialized:
            print("Not initialized")
            ready = False
        if self.node_robot_busy.read_value():
            print("Robot is busy")
            ready = False
        if not self.node_emergency_stop_ok.read_value():
            print("Not operational stop OK")
            ready = False
        if not self.node_operator_safety_ok.read_value():
            print("Not operator safety OK")
            ready = False
        if self.node_robot_coom_err.read_value():
            print("Comunication error")
            ready = False
        return ready

    def disconnect(self):
        self.opcua_client.disconnect()

if __name__ == "__main__":
    r = RobotControl()
    r.connect()

    p = r.get_position()
    print('Is data valid: ', p.Valid)
    print('X:', p.X)
    print('Y:', p.Y)
    print('Z:', p.Z)
    print('A:', p.A)
    print('B:', p.B)
    print('C:', p.C)
    print('Base:', p.Base)
    print('Tool:', p.Tool)

    r.disconnect()
