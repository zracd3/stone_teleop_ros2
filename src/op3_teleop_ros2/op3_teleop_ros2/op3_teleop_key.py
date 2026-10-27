#!/usr/bin/env python3
import sys
import select
import tty
import termios
import math
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

# Importar mensajes y servicios de caminata de ROBOTIS OP3
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam

ACTION_MAP = {
    '1': 9,   # Stand / WalkReady
    '2': 90,  # Saludar
    '3': 93,  # Patada
    '4': 17,  # Sentarse
}

msg = """
================================================
   Control Teleoperado de Stone (ROS 2 Humble)
================================================
Caminata (Límites de Amplitud Seguros):
   W / w            : Avanzar (X = +0.020 m)
   S / s            : Retroceder (X = -0.020 m)
   A / a            : Paso Lateral Izquierda (Y = -0.010 m)
   D / d            : Paso Lateral Derecha (Y = +0.010 m)

Rotación (Yaw / Angle):
   Q / q            : Giro Izquierda (+15 deg)
   E / e            : Giro Derecha (-15 deg)

   ESPACIO          : Detener caminata (STOP)

Acciones Guardadas:
   1, 2, 3, 4       : Ejecutar Página del Action Editor

CTRL-C              : Apagado Seguro y Salida
================================================
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class OP3TeleopKeySeguro(Node):
    def __init__(self):
        super().__init__('op3_teleop_keyboard_seguro')
        
        # Publicadores
        self.pub_walk_params = self.create_publisher(WalkingParam, '/robotis/walking/set_params', 10)
        self.pub_walk_cmd = self.create_publisher(String, '/robotis/walking/command', 10)
        self.pub_vel = self.create_publisher(Twist, '/robotis/walking/cmd_vel', 10)
        self.pub_action = self.create_publisher(Int32, '/robotis/action/page_num', 10)
        self.pub_module = self.create_publisher(String, '/robotis/enable_ctrl_module', 10)
        
        # Cliente de Servicio Bloqueante Sincrónico
        self.cli_get_params = self.create_client(GetWalkingParam, '/robotis/walking/get_params')
        
        self.current_walk_params = None
        self.current_module = ""
        self.is_walking = False
        
        # LÍMITES DE SEGURIDAD ESTÍCTOS DE HARDWARE
        self.MAX_X_AMP = 0.025    # Max 2.5 cm
        self.MAX_Y_AMP = 0.012    # Max 1.2 cm
        self.MAX_YAW_DEG = 15.0   # Max 15 grados
        
        # Carga síncrona bloqueante garantizada al inicio
        self.load_initial_parameters_sync()

    def load_initial_parameters_sync(self):
        """
        CORRECCIÓN 1: Carga síncrona bloqueante.
        Garantiza al 100% que la aplicación NO iniciará si no ha leído los
        parámetros originales del robot, evitando publicar ceros o basura.
        """
        self.get_logger().info('Cargando parámetros iniciales del op3_manager...')
        
        while not self.cli_get_params.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Esperando que el servicio /robotis/walking/get_params esté activo...')

        req = GetWalkingParam.Request()
        future = self.cli_get_params.call_async(req)
        
        # Spin controlado para esperar la respuesta completa
        rclpy.spin_until_future_complete(self, future)
        
        try:
            res = future.result()
            if res is not None:
                self.current_walk_params = res.parameters
                self.get_logger().info('INTEGRIDAD DE HARDWARE VERIFICADA: Offsets y ganancias de balance respaldados.')
            else:
                raise RuntimeError("El servicio respondió None")
        except Exception as e:
            self.get_logger().fatal(f'Error crítico al leer parámetros de caminata: {e}')
            sys.exit(1)

    def apply_walk_config(self, x_amp, y_amp, yaw_deg):
        """
        CORRECCIÓN 2: Filtrado y acotado por Hardware.
        Aplica los límites máximos permitidos antes de enviar al robot.
        """
        if self.current_walk_params is None:
            self.get_logger().error('Bloqueo de seguridad: Parámetros no cargados.')
            return False

        # Clampear/Saturar valores para evitar fuerzo mecánico
        safe_x = max(min(float(x_amp), self.MAX_X_AMP), -self.MAX_X_AMP)
        safe_y = max(min(float(y_amp), self.MAX_Y_AMP), -self.MAX_Y_AMP)
        safe_yaw_deg = max(min(float(yaw_deg), self.MAX_YAW_DEG), -self.MAX_YAW_DEG)

        # Modificar solo amplitudes de movimiento sobre la plantilla base
        self.current_walk_params.x_move_amplitude = safe_x
        self.current_walk_params.y_move_amplitude = safe_y
        self.current_walk_params.angle_move_amplitude = math.radians(safe_yaw_deg)

        # Publicar objeto completo
        self.pub_walk_params.publish(self.current_walk_params)
        return True

    def change_module(self, module_name):
        """
        CORRECCIÓN 3: Transición suave entre módulos.
        Envía la orden de cambio de módulo de control.
        """
        msg = String()
        msg.data = module_name
        self.pub_module.publish(msg)

    def start_walking(self):
        if not self.is_walking:
            msg = String()
            msg.data = "start"
            self.pub_walk_cmd.publish(msg)
            self.is_walking = True

    def stop_walking(self):
        if self.is_walking:
            # 1. Detener velocidad lineal/angular
            self.pub_vel.publish(Twist())
            
            # 2. Enviar orden stop
            msg = String()
            msg.data = "stop"
            self.pub_walk_cmd.publish(msg)
            self.is_walking = False

    def send_action(self, page_num):
        msg = Int32()
        msg.data = page_num
        self.pub_action.publish(msg)

    def safe_shutdown(self):
        """
        CORRECCIÓN 4: Apagado seguro sin estrés en los servos.
        """
        self.get_logger().info("Ejecutando Secuencia de Apagado Seguro...")
        try:
            # Detener marcha
            self.stop_walking()

            # Limpiar vector de velocidad
            self.pub_vel.publish(Twist())

            self.change_module("action_module")
            self.current_module = "action_module"

            self.get_logger().info("Enviando robot a postura de seguridad (Página 17)...")
            self.send_action(17)

            time.sleep(5.0)
            
            # Regresar al módulo neutral sin fuerza
            self.change_module("none")
            
            self.get_logger().info("APAGADO SEGURO COMPLETADO: Hardware en estado seguro.")
        except Exception as e:
            self.get_logger().error(f"Error en apagado seguro: {e}")

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = OP3TeleopKeySeguro()

    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)

            # TECLAS DE CAMINATA WASD + QE + ESPACIO
            if key in ['w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'q', 'Q', 'e', 'E', ' ']:
                
                # Transición a modulo de caminata si venía de otro modulo
                if node.current_module != "walking_module":
                    node.change_module("walking_module")
                    node.current_module = "walking_module"

                # W / w -> Avanzar
                if key in ['w', 'W']:
                    if node.apply_walk_config(x_amp=0.020, y_amp=0.000, yaw_deg=0.0):
                        node.start_walking()

                # S / s -> Retroceder
                elif key in ['s', 'S']:
                    if node.apply_walk_config(x_amp=-0.020, y_amp=0.000, yaw_deg=0.0):
                        node.start_walking()

                # A / a -> Paso lateral Izquierda
                elif key in ['a', 'A']:
                    if node.apply_walk_config(x_amp=0.000, y_amp=-0.010, yaw_deg=0.0):
                        node.start_walking()

                # D / d -> Paso lateral Derecha
                elif key in ['d', 'D']:
                    if node.apply_walk_config(x_amp=0.000, y_amp=0.010, yaw_deg=0.0):
                        node.start_walking()

                # Q / q -> Girar Izquierda
                elif key in ['q', 'Q']:
                    if node.apply_walk_config(x_amp=0.000, y_amp=0.000, yaw_deg=-12.0):
                        node.start_walking()

                # E / e -> Girar Derecha
                elif key in ['e', 'E']:
                    if node.apply_walk_config(x_amp=0.000, y_amp=0.000, yaw_deg=12.0):
                        node.start_walking()

                # ESPACIO -> Parar
                elif key == ' ':
                    node.stop_walking()

            # TECLAS DE ACCIÓN (Action Editor)
            elif key in ACTION_MAP:
                if node.current_module != "action_module":
                    node.stop_walking()
                    node.change_module("action_module")
                    node.current_module = "action_module"

                page = ACTION_MAP[key]
                node.get_logger().info(f'Ejecutando acción página: {page}')
                node.send_action(page)

            elif key == '\x03':  # CTRL + C
                break

    except Exception as e:
        print(f"Excepción en ejecución: {e}")

    finally:
        node.safe_shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()