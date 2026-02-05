#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

class ImageRectifier(Node):
    def __init__(self):
        super().__init__('image_rectifier_left')

        # Declarar parámetros con valores por defecto
        self.declare_parameter("gain_l", 0.85)
        self.declare_parameter("gain_v", 0.36)
        self.declare_parameter("gain_s", 0.01)
        self.declare_parameter("clahe_clip", 0.1)
        self.declare_parameter("blur_k", 3)
        
        # Agregar callback para cambios de parámetros
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.bridge = CvBridge()
        
        # Variables para mostrar ayuda
        self.show_help = True
        self.help_text = """
        CONTROLES DE TECLADO:
        ---------------------
        L: Incrementar ganancia L (+0.1)
        l: Decrementar ganancia L (-0.1)
        V: Incrementar ganancia V (+0.1)
        v: Decrementar ganancia V (-0.1)
        S: Incrementar ganancia S (+0.1)
        s: Decrementar ganancia S (-0.1)
        C: Incrementar CLAHE clip (+0.5)
        c: Decrementar CLAHE clip (-0.5)
        B: Incrementar blur kernel (+2)
        b: Decrementar blur kernel (-2)
        h: Mostrar/ocultar ayuda
        r: Resetear parámetros
        q: Salir
        
        PARÁMETROS ACTUALES:
        """
        
        # Suscripción a la imagen original
        self.sub_img = self.create_subscription(
            Image,
            '/camera/rgb/left',
            self.image_callback,
            10
        )

        # Publicador de imagen corregida
        self.pub_img = self.create_publisher(
            Image,
            '/camera/rgb/left_corrected',
            10
        )

        # Ventana para mostrar la imagen
        #cv2.namedWindow('Imagen Corregida', cv2.WINDOW_NORMAL)
        #cv2.resizeWindow('Imagen Corregida', 800, 600)
        
        # Configurar callback de teclado
        #cv2.setMouseCallback('Imagen Corregida', self.on_mouse)

    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parámetro {param.name} cambiado a {param.value}")
        return SetParametersResult(successful=True)

    def on_mouse(self, event, x, y, flags, param):
        # Callback para eventos del mouse (puede expandirse si se necesita)
        pass

    def update_parameters_from_keyboard(self, key):
        """Actualiza parámetros según tecla presionada"""
        params_changed = False
        
        # Obtener valores actuales
        gain_l = self.get_parameter("gain_l").value
        gain_v = self.get_parameter("gain_v").value
        gain_s = self.get_parameter("gain_s").value
        clahe_clip = self.get_parameter("clahe_clip").value
        blur_k = self.get_parameter("blur_k").value
        
        # Procesar teclas
        if key == ord('l'):
            gain_l = min(gain_l + 0.01, 10.0)
            params_changed = True
        elif key == ord('k'):
            gain_l = max(gain_l - 0.01, 0.01)
            params_changed = True
        elif key == ord('v'):
            gain_v = min(gain_v + 0.01, 10.0)
            params_changed = True
        elif key == ord('c'):
            gain_v = max(gain_v - 0.01, 0.01)
            params_changed = True
        elif key == ord('s'):
            gain_s = min(gain_s + 0.01, 10.0)
            params_changed = True
        elif key == ord('a'):
            gain_s = max(gain_s - 0.01, 0.01)
            params_changed = True
        elif key == ord('x'):
            clahe_clip = min(clahe_clip + 0.05, 40.0)
            params_changed = True
        elif key == ord('z'):
            clahe_clip = max(clahe_clip - 0.05, 0.05)
            params_changed = True
        elif key == ord('n'):
            blur_k = min(blur_k + 2, 31)
            # Asegurar que sea impar
            if blur_k % 2 == 0:
                blur_k += 1
            params_changed = True
        elif key == ord('b'):
            blur_k = max(blur_k - 2, 1)
            # Asegurar que sea impar
            if blur_k % 2 == 0:
                blur_k += 1
            params_changed = True
        elif key == ord('r'):
            # Resetear a valores por defecto
            gain_l = 1.0
            gain_v = 1.0
            gain_s = 1.0
            clahe_clip = 2.0
            blur_k = 3
            params_changed = True
        elif key == ord('h'):
            self.show_help = not self.show_help
            self.get_logger().info(f"Ayuda {'mostrada' if self.show_help else 'ocultada'}")
        
        # Actualizar parámetros si hubo cambios
        if params_changed:
            self.set_parameters([
                rclpy.parameter.Parameter('gain_l', rclpy.Parameter.Type.DOUBLE, gain_l),
                rclpy.parameter.Parameter('gain_v', rclpy.Parameter.Type.DOUBLE, gain_v),
                rclpy.parameter.Parameter('gain_s', rclpy.Parameter.Type.DOUBLE, gain_s),
                rclpy.parameter.Parameter('clahe_clip', rclpy.Parameter.Type.DOUBLE, clahe_clip),
                rclpy.parameter.Parameter('blur_k', rclpy.Parameter.Type.INTEGER, blur_k)
            ])
            
            self.get_logger().info(
                f"Parámetros actualizados: L={gain_l:.1f}, V={gain_v:.1f}, "
                f"S={gain_s:.1f}, CLAHE={clahe_clip:.1f}, Blur={blur_k}"
            )
        
        return key == ord('q')  # Retorna True si se presionó 'q'

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Recuperar parámetros
        gain_l = self.get_parameter("gain_l").value
        gain_v = self.get_parameter("gain_v").value
        gain_s = self.get_parameter("gain_s").value
        clahe_clip = self.get_parameter("clahe_clip").value
        blur_k = self.get_parameter("blur_k").value

        # ====================
        # CORRECCIÓN LAB-L
        # ====================
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(8, 8)
        )
        l = clahe.apply(l)

        # Escalado con centro (NO clip duro)
        l_f = l.astype(np.float32)
        l_mean = np.mean(l_f)
        l_corr = (l_f - l_mean) * gain_l + l_mean
        l_corr = np.clip(l_corr, 0, 255).astype(np.uint8)

        lab_corr = cv2.merge([l_corr, a, b])
        img_lab = cv2.cvtColor(lab_corr, cv2.COLOR_LAB2BGR)


        # ====================
        # CORRECCIÓN HSV
        # ====================
        hsv = cv2.cvtColor(img_lab, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Saturación controlada
        s_corr = cv2.addWeighted(
            s.astype(np.float32), gain_s,
            s.astype(np.float32), 0,
            0
        )

        # Brillo suave (no multiplicativo)
        v_corr = cv2.addWeighted(
            v.astype(np.float32), gain_v,
            v.astype(np.float32), 0,
            0
        )

        s_corr = np.clip(s_corr, 0, 255).astype(np.uint8)
        v_corr = np.clip(v_corr, 0, 255).astype(np.uint8)

        hsv_corr = cv2.merge([h, s_corr, v_corr])
        img_corr = cv2.cvtColor(hsv_corr, cv2.COLOR_HSV2BGR)


        # ====================
        # BLUR SUAVE SI ES NECESARIO
        # ====================
        if blur_k >= 3 and blur_k % 2 == 1:
            img_corr = cv2.GaussianBlur(img_corr, (blur_k, blur_k), 0)

        # ====================
        # MOSTRAR IMAGEN CON INFORMACIÓN
        # ====================
        display_img = img_corr.copy()
        
        if self.show_help:
            # Crear overlay para texto
            overlay = display_img.copy()
            cv2.rectangle(overlay, (0, 0), (600, 280), (0, 0, 0), -1)
            display_img = cv2.addWeighted(overlay, 0.7, display_img, 0.3, 0)
            
            # Texto con parámetros actuales
            params_text = f"""
            PARÁMETROS ACTUALES:
            Gain L: {gain_l:.2f}     (l/k)
            Gain V: {gain_v:.2f}     (v/c)
            Gain S: {gain_s:.2f}     (s/a)
            CLAHE Clip: {clahe_clip:.1f} (x/z)
            Blur Kernel: {blur_k}     (n/b)
            """
            
            # Dividir texto en líneas
            lines = params_text.strip().split('\n')
            for i, line in enumerate(lines):
                y_pos = 30 + i * 25
                cv2.putText(display_img, line.strip(), (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Instrucciones
            cv2.putText(display_img, "Presiona 'h' para ocultar ayuda", 
                       (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(display_img, "Presiona 'r' para resetear", 
                       (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(display_img, "Presiona 'q' para salir", 
                       (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        else:
            # Mostrar solo indicador mínimo
            cv2.putText(display_img, "Presiona 'h' para ayuda", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Mostrar imagen
        #cv2.imshow('Imagen Corregida', display_img)
        
        # Procesar teclado (espera 1ms para no bloquear)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:  # Si se presionó una tecla
            if self.update_parameters_from_keyboard(key):
                self.get_logger().info("Saliendo...")
                cv2.destroyAllWindows()
                rclpy.shutdown()
                return

        # ====================
        # PUBLICAR
        # ====================
        try:
            msg_out = self.bridge.cv2_to_imgmsg(img_corr, encoding="bgr8")
            msg_out.header = msg.header
            self.pub_img.publish(msg_out)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge publish error: {e}")

    def destroy_node(self):
        """Limpieza al destruir el nodo"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            cv2.destroyAllWindows()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()