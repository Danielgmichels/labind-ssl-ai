import socket
import ssl_simulation_robot_control_pb2
# ==========================================
# MÓDULO 3: AÇÃO (Rede UDP) - LOCAL VELOCITY
# ==========================================
class ActionClient:
    def __init__(self, ip="127.0.0.1", port=10302):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Atualizamos kick_z_speed para kick_angle, seguindo o protocolo oficial da SSL
    def send_command(self, robot_id, v_forward, v_left, vw, kick_speed=0.0, kick_angle=0.0, dribbler_speed=0.0):
        packet = ssl_simulation_robot_control_pb2.RobotControl()
        command = packet.robot_commands.add()
        command.id = robot_id
        
        command.move_command.local_velocity.forward = v_forward
        command.move_command.local_velocity.left = v_left
        command.move_command.local_velocity.angular = vw
        
        # O Chute (Força em m/s)
        command.kick_speed = kick_speed
        
        # O Ângulo do chute (0 para rasteiro, >0 para cavadinha/cruzamento)
        command.kick_angle = kick_angle
        
        # O Driblador (Rolo giratório que gruda a bola no bico do robô)
        command.dribbler_speed = dribbler_speed

        data = packet.SerializeToString()
        self.sock.sendto(data, (self.ip, self.port))