#!/usr/bin/env python3
"""
system_health_monitor.py
Nodo ROS2 que monitorea:
- Temperatura GPU
- Temperatura CPU  
- Batería
- RAM/CPU usage
- Temperatura disco
Publica en topics individuales DiagnosticStatus
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import psutil
import subprocess
import os
import time
from datetime import datetime
import glob
from std_msgs.msg import Bool


class SystemHealthMonitor(Node):
    def __init__(self):
        super().__init__('system_health_monitor')
        
        # Publishers individuales para cada tipo de diagnóstico
        self.cpu_temp_pub = self.create_publisher(DiagnosticStatus, '/status/cpu_temperature', 10)
        self.gpu_temp_pub = self.create_publisher(DiagnosticStatus, '/status/gpu_temperature', 10)
        self.battery_pub = self.create_publisher(DiagnosticStatus, '/status/battery_laptop', 10)
        self.ram_pub = self.create_publisher(DiagnosticStatus, '/status/ram', 10)
        self.cpu_usage_pub = self.create_publisher(DiagnosticStatus, '/status/cpu_usage', 10)
        self.gpu_usage_pub = self.create_publisher(DiagnosticStatus, '/status/gpu_usage', 10)
        self.disk_temp_pub = self.create_publisher(DiagnosticStatus, '/status/disk_temperature', 10)
        self.uptime_pub = self.create_publisher(DiagnosticStatus, '/status/uptime', 10)
        self.system_summary_pub = self.create_publisher(DiagnosticStatus, '/status/system_summary', 10)

        # Publisher para estado de la tapa (lid)
        self.lid_pub = self.create_publisher(Bool, '/lid_closed', 10)

        
        # Limites de seguridad 
        self.limits = {
            'cpu_temp_warning': 85.0,    # °C – Advertencia cuando CPU empieza a estar “caliente”.
            'cpu_temp_critical': 95.0,   # °C – Crítico: cerca del límite seguro de muchos CPUs modernos en laptops.
            'gpu_temp_warning': 80.0,    # °C – Advertencia cuando GPU se calienta bajo carga.
            'gpu_temp_critical': 90.0,   # °C – Crítico: límite razonable antes de posible throttling o daño.
            'disk_temp_warning': 55.0,   # °C – Advertencia para el disco (si es SSD/NVMe, mantener moderado).
            'battery_warning': 25.0,     # % – Warning.
            'battery_critical': 15.0,    # % – Critical.
            'cpu_usage_warning': 90.0,   # % – Warning indicar cuando CPU esté saturada.
            'ram_usage_warning': 90.0,   # % – Warning.
        }

        
        # Estado actual
        self.current_state = {
            'cpu_temp': 0.0,
            'gpu_temp': 0.0,
            'battery_percent': 100.0,
            'battery_plugged': True,
            'cpu_usage': 0.0,
            'gpu_usage': 0.0,   # Solo para NVIDIA
            'ram_usage': 0.0,
            'ram_used_gb': 0.0,
            'ram_total_gb': 0.0,
            'disk_temp': 0.0,
            'disk_usage': 0.0,
            'disk_free_gb': 0.0,
            'uptime': 0.0,
            'fan_rpm': 0
        }
        
        # Historial para promedios
        self.temp_history = {
            'cpu': [],
            'gpu': [],
            'max_history': 10
        }
        
        # Timer para publicación periódica
        self.timer = self.create_timer(2.0, self.publish_diagnostics)  # Cada 2 segundos
        
        # Timer para checks de seguridad (más lento)
        self.safety_timer = self.create_timer(5.0, self.safety_checks)
        
        #self.get_logger().info("System Health Monitor iniciado (topics individuales)")
        
        # Detectar tipo de GPU (NVIDIA/Intel/AMD)
        self.gpu_type = self.detect_gpu_type()
        #self.get_logger().info(f"GPU detectada: {self.gpu_type}")
    
    def detect_gpu_type(self):
        """Detecta el tipo de GPU disponible"""
        try:
            # Verificar si es NVIDIA
            result = subprocess.run(['which', 'nvidia-smi'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                return "nvidia"
            
            # Verificar si es Intel
            result = subprocess.run(['which', 'intel_gpu_top'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                return "intel"
            
        except Exception:
            pass
        
        return "unknown"
    
    def get_cpu_temperature(self):
        """Obtiene temperatura de la CPU en °C"""
        try:
            # Intentar leer de múltiples thermal zones
            max_temp = 0.0
            for i in range(20):  # Buscar en múltiples zonas
                zone_path = f'/sys/class/thermal/thermal_zone{i}'
                temp_path = f'{zone_path}/temp'
                
                if os.path.exists(temp_path):
                    with open(temp_path, 'r') as f:
                        temp = float(f.read().strip()) / 1000.0
                        if temp > max_temp:
                            max_temp = temp
            
            if max_temp > 0:
                return max_temp
            
            # Alternativa: usar sensors command
            try:
                result = subprocess.run(['sensors'], capture_output=True, text=True, timeout=1)
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'Core' in line and '°C' in line:
                            parts = line.split('+')
                            if len(parts) > 1:
                                temp_str = parts[1].split('°')[0]
                                temp = float(temp_str)
                                if temp > max_temp:
                                    max_temp = temp
            except:
                pass
            
        except Exception as e:
            #self.get_logger().warn(f"No se pudo leer temp CPU: {e}")
            pass
        
        # Fallback: estimar por uso de CPU
        if max_temp == 0:
            cpu_usage = psutil.cpu_percent(interval=0.1)
            max_temp = 40.0 + (cpu_usage * 0.3)
        
        return max_temp
    
    def get_gpu_temperature(self):
        """Obtiene temperatura de la GPU en °C"""
        try:
            if self.gpu_type == "nvidia":
                # Usar nvidia-smi para NVIDIA
                result = subprocess.run(
                    ['nvidia-smi', '--query-gpu=temperature.gpu', '--format=csv,noheader'],
                    capture_output=True, text=True, timeout=2
                )
                if result.returncode == 0 and result.stdout.strip():
                    return float(result.stdout.strip())
            
            # Método genérico para Linux
            for hwmon in range(5):
                hwmon_dir = f'/sys/class/hwmon/hwmon{hwmon}'
                name_path = f'{hwmon_dir}/name'
                temp_path = f'{hwmon_dir}/temp1_input'
                
                if os.path.exists(name_path) and os.path.exists(temp_path):
                    with open(name_path, 'r') as f:
                        name = f.read().strip().lower()
                    
                    if 'gpu' in name or 'radeon' in name or 'amdgpu' in name:
                        with open(temp_path, 'r') as f:
                            return float(f.read().strip()) / 1000.0
        
        except Exception as e:
            #self.get_logger().warn(f"No se pudo leer temp GPU: {e}")
            pass
        
        return 0.0
    
    def get_battery_info(self):
        """Obtiene información de la batería"""
        try:
            battery = psutil.sensors_battery()
            if battery:
                return {
                    'percent': battery.percent,
                    'plugged': battery.power_plugged,
                    'time_left': battery.secsleft if battery.secsleft > 0 else None
                }
        except Exception as e:
            #self.get_logger().warn(f"No se pudo leer batería: {e}")
            pass
        
        return {'percent': 100.0, 'plugged': True, 'time_left': None}
    
    def get_disk_temperature(self):
        """Obtiene temperatura disco sin sudo"""
        try:
            # Método 1: NVMe sysfs
            nvme_temp_files = glob.glob('/sys/class/nvme/nvme*/device/hwmon/hwmon*/temp*_input')
            for temp_file in nvme_temp_files:
                try:
                    with open(temp_file, 'r') as f:
                        return float(f.read().strip()) / 1000.0
                except:
                    continue
            
            # Método 2: SATA/block devices
            block_temp_files = glob.glob('/sys/class/block/*/device/hwmon/hwmon*/temp*_input')
            for temp_file in block_temp_files:
                try:
                    with open(temp_file, 'r') as f:
                        return float(f.read().strip()) / 1000.0
                except:
                    continue
            
            # Método 3: hwmon general
            hwmon_temp_files = glob.glob('/sys/class/hwmon/hwmon*/temp*_input')
            for temp_file in hwmon_temp_files:
                try:
                    # Verificar si es un sensor de disco (por nombre del dispositivo)
                    base_dir = os.path.dirname(temp_file)
                    name_file = os.path.join(base_dir, 'name')
                    if os.path.exists(name_file):
                        with open(name_file, 'r') as f:
                            name = f.read().strip().lower()
                            if 'nvme' in name or 'sata' in name or 'ahci' in name:
                                with open(temp_file, 'r') as f:
                                    return float(f.read().strip()) / 1000.0
                except:
                    continue
            
        except Exception as e:
            self.get_logger().debug(f"Error temp disco: {e}")
        
        # Valor por defecto razonable
        return 45.0
    
    def get_fan_speed(self):
        """Obtiene velocidad del ventilador"""
        try:
            for hwmon in range(5):
                fan_path = f'/sys/class/hwmon/hwmon{hwmon}/fan1_input'
                if os.path.exists(fan_path):
                    with open(fan_path, 'r') as f:
                        return int(f.read().strip())
        except:
            pass
        return 0
    
    def get_gpu_usage_nvidia(self):
        """Obtiene uso de GPU usando gpustat (más preciso)"""
        try:
            result = subprocess.run(
                ['gpustat', '--no-color', '--no-header'],
                capture_output=True, text=True, timeout=2
            )
            if result.returncode == 0:
                # Ejemplo: "0  GeForce RTX 3060 ... | 35°C,  6 %"
                parts = result.stdout.strip().split('|')
                if len(parts) > 1:
                    usage_part = parts[1].split(',')
                    if len(usage_part) > 1:
                        usage_str = usage_part[1].strip().split()[0]  # "6 %" -> "6"
                        return float(usage_str)
        except:
            pass
        return 0.0
    
    def get_system_stats(self):
        """Obtiene estadísticas del sistema"""
        try:
            cpu_usage = psutil.cpu_percent(interval=0.1)
            ram = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            uptime = time.time() - psutil.boot_time()
            
            return {
                'cpu_usage': cpu_usage,
                'ram_usage': ram.percent,
                'ram_total_gb': ram.total / (1024**3),
                'ram_used_gb': ram.used / (1024**3),
                'disk_usage': disk.percent,
                'disk_free_gb': disk.free / (1024**3),
                'uptime_hours': uptime / 3600
            }
        except Exception as e:
            #self.get_logger().warn(f"Error obteniendo stats: {e}")
            return {}
    
    def get_lid_state(self):
        """Detecta si la tapa de la laptop está cerrada."""
        try:
            lid_paths = glob.glob('/proc/acpi/button/lid/*/state')
            if lid_paths:
                with open(lid_paths[0], 'r') as f:
                    content = f.read().lower()
                    if "closed" in content:
                        return True
                    return False
        except:
            pass
        return False

    
    def update_current_state(self):
        """Actualiza todos los estados del sistema"""
        try:
            # Temperaturas
            self.current_state['cpu_temp'] = self.get_cpu_temperature()
            self.current_state['gpu_temp'] = self.get_gpu_temperature()
            self.current_state['disk_temp'] = self.get_disk_temperature()
            self.current_state['fan_rpm'] = self.get_fan_speed()
            
            # Historial
            self.temp_history['cpu'].append(self.current_state['cpu_temp'])
            self.temp_history['gpu'].append(self.current_state['gpu_temp'])
            
            if len(self.temp_history['cpu']) > self.temp_history['max_history']:
                self.temp_history['cpu'].pop(0)
                self.temp_history['gpu'].pop(0)
            
            # Batería
            battery_info = self.get_battery_info()
            self.current_state['battery_percent'] = battery_info['percent']
            self.current_state['battery_plugged'] = battery_info['plugged']
            self.current_state['battery_time_left'] = battery_info['time_left']

            # Después de obtener gpu_temp
            if self.gpu_type == "nvidia":
                self.current_state['gpu_usage'] = self.get_gpu_usage_nvidia()
            elif self.gpu_type == "intel":
                self.current_state['gpu_usage'] = self.get_gpu_usage_intel()
            
            # Estadísticas
            stats = self.get_system_stats()
            if stats:
                self.current_state.update(stats)
            
        except Exception as e:
            #self.get_logger().error(f"Error actualizando estado: {e}")
            pass
    
    def safety_checks(self):
        """Realiza checks de seguridad"""
        cpu_temp = self.current_state['cpu_temp']
        gpu_temp = self.current_state['gpu_temp']
        battery = self.current_state['battery_percent']
        plugged = self.current_state['battery_plugged']
        
        """if cpu_temp > self.limits['cpu_temp_critical']:
            self.get_logger().error(f"TEMPERATURA CPU CRÍTICA: {cpu_temp:.1f}°C")
        
        if gpu_temp > self.limits['gpu_temp_critical'] and gpu_temp > 0:
            self.get_logger().error(f"TEMPERATURA GPU CRÍTICA: {gpu_temp:.1f}°C")
        
        if not plugged and battery < self.limits['battery_critical']:
            self.get_logger().error(f"BATERÍA CRÍTICA: {battery:.1f}%")"""
    
    def create_diagnostic_message(self, name, hardware_id, level, message, values=None):
        """Crea un mensaje DiagnosticStatus individual"""
        diag = DiagnosticStatus()
        diag.name = name
        diag.hardware_id = hardware_id
        diag.level = level
        diag.message = message
        
        if values:
            for key, value in values:
                kv = KeyValue()
                kv.key = key
                kv.value = value
                diag.values.append(kv)
        
        return diag
    
    def publish_diagnostics(self):
        """Publica cada diagnóstico en su topic individual"""
        self.update_current_state()
        
        # 1. Temperatura CPU
        cpu_temp_diag = self.create_diagnostic_message(
            name="Temperatura del CPU",
            hardware_id="laptop_cpu",
            level=self.get_temp_level(self.current_state['cpu_temp'], 'cpu'),
            message=f"{self.current_state['cpu_temp']:.1f}°C",
            values=[
                ("temperature", f"{self.current_state['cpu_temp']:.1f}"),
                ("unit", "°C"),
                ("warning_limit", f"{self.limits['cpu_temp_warning']}"),
                ("critical_limit", f"{self.limits['cpu_temp_critical']}"),
                ("average_5s", f"{self.get_average_temp('cpu'):.1f}")
            ]
        )
        self.cpu_temp_pub.publish(cpu_temp_diag)
        
        # 2. Temperatura GPU (si disponible)
        if self.current_state['gpu_temp'] > 0:
            gpu_temp_diag = self.create_diagnostic_message(
                name="Temperatura de la GPU",
                hardware_id=f"laptop_gpu_{self.gpu_type}",
                level=self.get_temp_level(self.current_state['gpu_temp'], 'gpu'),
                message=f"{self.current_state['gpu_temp']:.1f}°C ({self.gpu_type})",
                values=[
                    ("temperature", f"{self.current_state['gpu_temp']:.1f}"),
                    ("unit", "°C"),
                    ("type", self.gpu_type),
                    ("warning_limit", f"{self.limits['gpu_temp_warning']}"),
                    ("critical_limit", f"{self.limits['gpu_temp_critical']}")
                ]
            )
            self.gpu_temp_pub.publish(gpu_temp_diag)
        
        # 3. Batería
        battery_diag = self.create_diagnostic_message(
            name="Porcentaje de la Bateria Laptop",
            hardware_id="laptop_battery",
            level=self.get_battery_level(self.current_state['battery_percent']),
            message=f"{self.current_state['battery_percent']:.1f}% {'(cargando)' if self.current_state['battery_plugged'] else '(descarga)'}",
            values=[
                ("percentage", f"{self.current_state['battery_percent']:.1f}"),
                ("unit", "%"),
                ("plugged", "yes" if self.current_state['battery_plugged'] else "no"),
                ("warning_level", f"{self.limits['battery_warning']}"),
                ("critical_level", f"{self.limits['battery_critical']}")
            ]
        )
        self.battery_pub.publish(battery_diag)
        
        # 4. Uso de RAM
        if 'ram_usage' in self.current_state:
            ram_diag = self.create_diagnostic_message(
                name="Uso de memoria ram",
                hardware_id="laptop_ram",
                level=self.get_usage_level(self.current_state['ram_usage']),
                message=f"{self.current_state['ram_usage']:.1f}% usada",
                values=[
                    ("usage_percent", f"{self.current_state['ram_usage']:.1f}"),
                    ("used_gb", f"{self.current_state.get('ram_used_gb', 0):.1f}"),
                    ("total_gb", f"{self.current_state.get('ram_total_gb', 0):.1f}"),
                    ("warning_level", f"{self.limits['ram_usage_warning']}")
                ]
            )
            self.ram_pub.publish(ram_diag)
        
        # 5. Uso de CPU
        if 'cpu_usage' in self.current_state:
            cpu_usage_diag = self.create_diagnostic_message(
                name="Uso de CPU",
                hardware_id="laptop_cpu_usage",
                level=self.get_usage_level(self.current_state['cpu_usage']),
                message=f"{self.current_state['cpu_usage']:.1f}%",
                values=[
                    ("usage_percent", f"{self.current_state['cpu_usage']:.1f}"),
                    ("temperature", f"{self.current_state['cpu_temp']:.1f}"),
                    ("warning_level", f"{self.limits['cpu_usage_warning']}")
                ]
            )
            self.cpu_usage_pub.publish(cpu_usage_diag)

        # 6. Uso de GPU (NVIDIA)
        if 'gpu_usage' in self.current_state and self.current_state['gpu_usage'] > 0:
            gpu_usage_diag = self.create_diagnostic_message(
                name="Uso de GPU",
                hardware_id=f"laptop_gpu_usage_{self.gpu_type}",
                level=self.get_usage_level(self.current_state['gpu_usage']),
                message=f"{self.current_state['gpu_usage']:.1f}%",
                values=[
                    ("usage_percent", f"{self.current_state['gpu_usage']:.1f}"),
                    ("type", self.gpu_type),
                    ("temperature", f"{self.current_state['gpu_temp']:.1f}")
                ]
            )
            self.gpu_usage_pub.publish(gpu_usage_diag)
        
        # 7. Temperatura Disco
        disk_temp_diag = self.create_diagnostic_message(
            name="Temperatura del disco",
            hardware_id="laptop_disk",
            level=self.get_temp_level(self.current_state['disk_temp'], 'disk'),
            message=f"{self.current_state['disk_temp']:.1f}°C",
            values=[
                ("temperature", f"{self.current_state['disk_temp']:.1f}"),
                ("unit", "°C"),
                ("warning_limit", f"{self.limits['disk_temp_warning']}"),
                ("usage", f"{self.current_state.get('disk_usage', 0):.1f}%")
            ]
        )
        self.disk_temp_pub.publish(disk_temp_diag)
        
        # 8. Uptime
        if 'uptime' in self.current_state:
            uptime_diag = self.create_diagnostic_message(
                name="Tiempo de uso",
                hardware_id="laptop_system",
                level=DiagnosticStatus.OK,
                message=f"{self.current_state['uptime']:.1f} hours",
                values=[
                    ("uptime_hours", f"{self.current_state['uptime']:.1f}"),
                    ("timestamp", datetime.now().isoformat())
                ]
            )
            self.uptime_pub.publish(uptime_diag)
        
        # 9. Resumen del sistema (opcional)
        summary_diag = self.create_diagnostic_message(
            name="System Health Summary",
            hardware_id="laptop_overall",
            level=self.get_overall_level(),
            message=self.get_summary_message(),
            values=[
                ("cpu_temp", f"{self.current_state['cpu_temp']:.1f}"),
                ("gpu_temp", f"{self.current_state['gpu_temp']:.1f}"),
                ("battery", f"{self.current_state['battery_percent']:.1f}"),
                ("cpu_usage", f"{self.current_state.get('cpu_usage', 0):.1f}"),
                ("ram_usage", f"{self.current_state.get('ram_usage', 0):.1f}"),
                ("fan_rpm", f"{self.current_state['fan_rpm']}")
            ]
        )
        self.system_summary_pub.publish(summary_diag)

        # 10. Estado de la tapa (lid)
        lid_closed = self.get_lid_state()
        lid_msg = Bool()
        lid_msg.data = lid_closed
        self.lid_pub.publish(lid_msg)   

        
        # Log periódico
        """if int(time.time()) % 10 == 0:  # Cada 10 segundos
            self.get_logger().info(
                f"Health: CPU {self.current_state['cpu_temp']:.1f}°C, "
                f"GPU {self.current_state['gpu_temp']:.1f}°C, "
                f"Bat {self.current_state['battery_percent']:.1f}%"
            )"""
    
    def get_average_temp(self, component):
        """Obtiene temperatura promedio del historial"""
        history = self.temp_history.get(component, [])
        if history:
            return sum(history) / len(history)
        return 0.0
    
    def get_temp_level(self, temp, component='cpu'):
        """Determina nivel basado en temperatura"""
        if component == 'cpu':
            warn = self.limits['cpu_temp_warning']
            crit = self.limits['cpu_temp_critical']
        elif component == 'gpu':
            warn = self.limits['gpu_temp_warning']
            crit = self.limits['gpu_temp_critical']
        else:
            warn = self.limits['disk_temp_warning']
            crit = warn + 10.0
        
        if temp >= crit:
            return DiagnosticStatus.ERROR
        elif temp >= warn:
            return DiagnosticStatus.WARN
        else:
            return DiagnosticStatus.OK
    
    def get_battery_level(self, percent):
        """Determina nivel basado en batería"""
        if percent <= self.limits['battery_critical']:
            return DiagnosticStatus.ERROR
        elif percent <= self.limits['battery_warning']:
            return DiagnosticStatus.WARN
        else:
            return DiagnosticStatus.OK
    
    def get_usage_level(self, usage):
        """Determina nivel basado en uso"""
        if usage >= self.limits['ram_usage_warning']:
            return DiagnosticStatus.WARN
        else:
            return DiagnosticStatus.OK
    
    def get_overall_level(self):
        """Determina nivel general del sistema"""
        levels = []
        
        # CPU temp
        levels.append(self.get_temp_level(self.current_state['cpu_temp'], 'cpu'))
        
        # GPU temp
        if self.current_state['gpu_temp'] > 0:
            levels.append(self.get_temp_level(self.current_state['gpu_temp'], 'gpu'))
        
        # Battery
        if not self.current_state['battery_plugged']:
            levels.append(self.get_battery_level(self.current_state['battery_percent']))
        
        # Si hay ERROR en alguno, retornar ERROR
        if DiagnosticStatus.ERROR in levels:
            return DiagnosticStatus.ERROR
        # Si hay WARN en alguno, retornar WARN
        elif DiagnosticStatus.WARN in levels:
            return DiagnosticStatus.WARN
        else:
            return DiagnosticStatus.OK
    
    def get_summary_message(self):
        """Crea mensaje de resumen"""
        messages = []
        
        if self.current_state['cpu_temp'] >= self.limits['cpu_temp_warning']:
            messages.append(f"CPU hot: {self.current_state['cpu_temp']:.1f}°C")
        
        if self.current_state['gpu_temp'] >= self.limits['gpu_temp_warning']:
            messages.append(f"GPU hot: {self.current_state['gpu_temp']:.1f}°C")
        
        if not self.current_state['battery_plugged']:
            if self.current_state['battery_percent'] < self.limits['battery_warning']:
                messages.append(f"Battery low: {self.current_state['battery_percent']:.1f}%")
        
        if messages:
            return "; ".join(messages)
        else:
            return "System healthy"
        
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SystemHealthMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown


if __name__ == '__main__':
    main()