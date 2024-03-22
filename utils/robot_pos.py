import asyncua.sync as opcua
import asyncua.ua as uatype
import logging
import time

PLC_OPCUA_URL = 'opc.tcp://10.100.0.120:4840/'
ROBOT_NAMESPACE_URI = 'http://robots.pickplace.ti40.cz'

class RobotPos():
    def connect(self):
        self.opcua_client = opcua.Client(PLC_OPCUA_URL)
        self.opcua_client.connect()
        self.types = self.opcua_client.load_data_type_definitions()
        self.ns_idx = self.opcua_client.get_namespace_index(ROBOT_NAMESPACE_URI)

        self.Node_Robot = self.opcua_client.get_node(f'ns={self.ns_idx};s=Cell.Robots.R32')
        self.Node_Robot_Position = self.opcua_client.get_node(f'ns={self.ns_idx};s=Cell.Robots.R32.Status.Position')
        self.Node_Robot_Busy = self.opcua_client.get_node(f'ns={self.ns_idx};s=Cell.Robots.R32.Status.Busy')
    
    def get_position(self):
        return self.Node_Robot_Position.read_value()

    def disconnect(self):
        self.opcua_client.disconnect()

if __name__ == "__main__":
    r = RobotPos()
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
