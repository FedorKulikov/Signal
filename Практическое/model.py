import tkinter as tk
from tkinter import ttk, scrolledtext
import socket, struct, time, random, threading
import queue


CONFIG = {
    "kp": 1.5,
    "ki": 1.5,
    "kd": 0.01,

    "noise_enabled": True,
    "noise_amplitude": 1.2,
    "noise_type": "gaussian", # "uniform" or "gaussian"

    "outlier_enabled": True,
    "outlier_amplitude": 5.0,
    "outlier_max_duration": 0.5,
    "outlier_frequency": 1.0,

    "send_period": 0.02,
    "send_ip": "127.0.0.1",
    "send_port": 50006,
    "receive_ip": "127.0.0.1",
    "receive_port": 50005
}


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error, self.integral = 0, 0
        self.integral_limit = 10.0
        
    def update(self, setpoint, current_value, dt):
        error = setpoint - current_value
        
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        
        if dt > 1e-6:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0
            
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class DynamicSystemApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Модель")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.debug_outliers = False
        self.log_queue = queue.Queue()
        
        self.lock = threading.Lock()
        self.current_value, self.setpoint = 0.0, 0.0
        self.noisy_value = 0.0
        self.start_time = time.time()
        
        self.sim_params = {
            "kp": CONFIG["kp"],
            "ki": CONFIG["ki"],
            "kd": CONFIG["kd"],
            "noise_enabled": CONFIG["noise_enabled"],
            "noise_amplitude": CONFIG["noise_amplitude"],
            "noise_type": CONFIG["noise_type"],
            "outlier_enabled": CONFIG["outlier_enabled"],
            "outlier_amplitude": CONFIG["outlier_amplitude"],
            "outlier_max_duration": CONFIG["outlier_max_duration"] * 1000,
            "outlier_frequency": CONFIG["outlier_frequency"],
            "send_period": CONFIG["send_period"] * 1000
        }
        
        self.active_outlier = {
            "active": False,
            "remaining_time": 0,
            "value": 0
        }
        
        self.send_ip = CONFIG["send_ip"]
        self.send_port = CONFIG["send_port"]
        self.receive_ip = CONFIG["receive_ip"]
        self.receive_port = CONFIG["receive_port"]
        
        self.pid = PID(CONFIG["kp"], CONFIG["ki"], CONFIG["kd"])
        
        self.running = True
        self.sock = None
        
        self.setup_ui()
        
        self.init_network()
        
        self.process_log_queue()
        
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_thread.start()
        
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()
        
        self.update_params_from_gui()
        
        self.update_display()
    
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        top_frame = ttk.LabelFrame(main_frame, text="Текущее состояние системы", padding="10")
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        value_frame = ttk.Frame(top_frame)
        value_frame.pack(fill=tk.X)
        
        ttk.Label(value_frame, text="Текущее значение:", font=("Arial", 11)).pack(side=tk.LEFT, padx=(0, 10))
        self.current_value_label = ttk.Label(value_frame, text="0.000", font=("Arial", 16, "bold"), foreground="blue")
        self.current_value_label.pack(side=tk.LEFT, padx=(0, 30))
        
        ttk.Label(value_frame, text="Целевое значение:", font=("Arial", 11)).pack(side=tk.LEFT, padx=(0, 10))
        self.setpoint_label = ttk.Label(value_frame, text="0.000", font=("Arial", 16, "bold"), foreground="green")
        self.setpoint_label.pack(side=tk.LEFT)
        
        center_frame = ttk.Frame(main_frame)
        center_frame.pack(fill=tk.BOTH, expand=True)
        
        left_frame = ttk.LabelFrame(center_frame, text="Параметры системы", padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        pid_frame = ttk.LabelFrame(left_frame, text="Параметры PID", padding="10")
        pid_frame.pack(fill=tk.X, pady=(0, 10))
        
        kp_frame = ttk.Frame(pid_frame)
        kp_frame.pack(fill=tk.X, pady=2)
        ttk.Label(kp_frame, text="Kp (пропорциональный):", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.kp_var = tk.StringVar(value=self.sim_params["kp"])
        ttk.Spinbox(kp_frame, from_=0.0, to=100.0, increment=0.1, textvariable=self.kp_var, width=10).pack(side=tk.LEFT)        
        
        ki_frame = ttk.Frame(pid_frame)
        ki_frame.pack(fill=tk.X, pady=2)
        ttk.Label(ki_frame, text="Ki (интегральный):", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.ki_var = tk.StringVar(value=self.sim_params["ki"])
        ttk.Spinbox(ki_frame, from_=0.0, to=100.0, increment=0.01, textvariable=self.ki_var, width=10).pack(side=tk.LEFT)
        
        kd_frame = ttk.Frame(pid_frame)
        kd_frame.pack(fill=tk.X, pady=2)
        ttk.Label(kd_frame, text="Kd (дифференциальный):", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.kd_var = tk.StringVar(value=self.sim_params["kd"])
        ttk.Spinbox(kd_frame, from_=0.0, to=100.0, increment=0.01, textvariable=self.kd_var, width=10).pack(side=tk.LEFT)
        
        noise_frame = ttk.LabelFrame(left_frame, text="Управление шумом", padding="10")
        noise_frame.pack(fill=tk.X, pady=(0, 10))
        
        type_frame = ttk.Frame(noise_frame)
        type_frame.pack(fill=tk.X, pady=2)
        ttk.Label(type_frame, text="Тип шума:", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.noise_type_var = tk.StringVar(value="gaussian")
        ttk.Radiobutton(type_frame, text="Гауссов", variable=self.noise_type_var, 
                       value="gaussian").pack(side=tk.LEFT, padx=(0, 10))
        ttk.Radiobutton(type_frame, text="Белый", variable=self.noise_type_var, 
                       value="uniform").pack(side=tk.LEFT)
        
        amp_frame = ttk.Frame(noise_frame)
        amp_frame.pack(fill=tk.X, pady=2)
        ttk.Label(amp_frame, text="Амплитуда:", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.noise_amp_var = tk.StringVar(value=self.sim_params["noise_amplitude"])
        ttk.Spinbox(amp_frame, from_=0.0, to=10.0, increment=0.01, textvariable=self.noise_amp_var, width=9).pack(side=tk.LEFT)

        self.noise_enabled_var = tk.BooleanVar(value=self.sim_params["noise_enabled"])
        ttk.Checkbutton(noise_frame, text="Включить шум", variable=self.noise_enabled_var).pack(anchor=tk.W, pady=(5, 0))
        
        outlier_frame = ttk.LabelFrame(left_frame, text="Управление выбросами", padding="10")
        outlier_frame.pack(fill=tk.X, pady=(0, 10))
        
        out_amp_frame = ttk.Frame(outlier_frame)
        out_amp_frame.pack(fill=tk.X, pady=2)
        ttk.Label(out_amp_frame, text="Амплитуда:", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.outlier_amp_var = tk.StringVar(value=self.sim_params["outlier_amplitude"])
        ttk.Spinbox(out_amp_frame, from_=0.0, to=100.0, increment=0.1, textvariable=self.outlier_amp_var, width=9).pack(side=tk.LEFT)

        out_dur_frame = ttk.Frame(outlier_frame)
        out_dur_frame.pack(fill=tk.X, pady=2)
        ttk.Label(out_dur_frame, text="Макс. длительность (мс):", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.outlier_dur_var = tk.StringVar(value=self.sim_params["outlier_max_duration"])
        ttk.Spinbox(out_dur_frame, from_=1, to=10000, increment=10, textvariable=self.outlier_dur_var, width=9).pack(side=tk.LEFT)

        out_freq_frame = ttk.Frame(outlier_frame)
        out_freq_frame.pack(fill=tk.X, pady=2)
        ttk.Label(out_freq_frame, text="Частота (Гц):", width=25, anchor=tk.W).pack(side=tk.LEFT)
        self.outlier_freq_var = tk.StringVar(value=self.sim_params["outlier_frequency"])
        ttk.Spinbox(out_freq_frame, from_=0.0, to=1000.0, increment=0.1, textvariable=self.outlier_freq_var, width=9).pack(side=tk.LEFT)

        self.outlier_enabled_var = tk.BooleanVar(value=self.sim_params["outlier_enabled"])
        ttk.Checkbutton(outlier_frame, text="Включить выбросы", variable=self.outlier_enabled_var).pack(anchor=tk.W, pady=(5, 0))
        
        right_frame = ttk.LabelFrame(center_frame, text="Сетевые настройки", padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        recv_frame = ttk.LabelFrame(right_frame, text="Прием команд (UDP)", padding="10")
        recv_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(recv_frame, text="IP адрес:", anchor=tk.W).pack(fill=tk.X, pady=2)
        self.receive_ip_var = tk.StringVar(value=self.receive_ip)
        ttk.Entry(recv_frame, textvariable=self.receive_ip_var).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(recv_frame, text="Порт:", anchor=tk.W).pack(fill=tk.X, pady=2)
        self.receive_port_var = tk.StringVar(value=self.receive_port)
        ttk.Entry(recv_frame, textvariable=self.receive_port_var).pack(fill=tk.X, pady=(0, 10))
        
        send_frame = ttk.LabelFrame(right_frame, text="Отправка состояния (UDP)", padding="10")
        send_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(send_frame, text="IP адрес:", anchor=tk.W).pack(fill=tk.X, pady=2)
        self.send_ip_var = tk.StringVar(value=self.send_ip)
        ttk.Entry(send_frame, textvariable=self.send_ip_var).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(send_frame, text="Порт:", anchor=tk.W).pack(fill=tk.X, pady=2)
        self.send_port_var = tk.StringVar(value=self.send_port)
        ttk.Entry(send_frame, textvariable=self.send_port_var).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(send_frame, text="Период отправки (мс):", anchor=tk.W).pack(fill=tk.X, pady=2)
        self.send_period_var = tk.StringVar(value=self.sim_params["send_period"])
        ttk.Entry(send_frame, textvariable=self.send_period_var).pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(right_frame, text="Применить сетевые настройки", 
                  command=self.apply_network_settings).pack(fill=tk.X, pady=(0, 10))
        
        info_frame = ttk.LabelFrame(right_frame, text="Формат данных", padding="10")
        info_frame.pack(fill=tk.X)
        
        ttk.Label(info_frame, text="Прием: 4 байта (float) - Целевое значение", 
                 font=("Consolas", 9), foreground="gray").pack(anchor=tk.W, pady=2)
        ttk.Label(info_frame, text="Отправка: 8 байт", 
                 font=("Consolas", 9), foreground="gray").pack(anchor=tk.W, pady=2)
        ttk.Label(info_frame, text="  - 4 байта: timestamp (uint32, мс)", 
                 font=("Consolas", 9), foreground="gray").pack(anchor=tk.W, padx=(10, 0))
        ttk.Label(info_frame, text="  - 4 байта: значение (float)", 
                 font=("Consolas", 9), foreground="gray").pack(anchor=tk.W, padx=(10, 0))
        
        log_frame = ttk.LabelFrame(main_frame, text="Лог событий", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        
        self.log = scrolledtext.ScrolledText(log_frame, height=12, font=("Consolas", 9))
        self.log.pack(fill=tk.BOTH, expand=True)
        
        self.status_bar = ttk.Label(self.root, text="Система запущена. Ожидание команд...", 
                                   relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def init_network(self):
        """Инициализация сетевого соединения"""
        try:
            if self.sock:
                self.sock.close()
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.receive_ip, self.receive_port))
            self.sock.setblocking(False)
            self.add_log_message(f"Сокет инициализирован. Прием команд на {self.receive_ip}:{self.receive_port}")
            self.add_log_message(f"Отправка состояния на {self.send_ip}:{self.send_port}")
        except Exception as e:
            self.add_log_message(f"Ошибка инициализации сети: {e}")
            self.sock = None
    
    def update_params_from_gui(self):
        """Обновление параметров из GUI в thread-safe структуру"""
        try:
            with self.lock:
                try:
                    self.sim_params["kp"] = float(self.kp_var.get())
                except ValueError:
                    pass
                try:
                    self.sim_params["ki"] = float(self.ki_var.get())
                except ValueError:
                    pass
                try:
                    self.sim_params["kd"] = float(self.kd_var.get())
                except ValueError:
                    pass
                
                self.sim_params["noise_enabled"] = self.noise_enabled_var.get()
                self.sim_params["noise_type"] = self.noise_type_var.get()
                try:
                    self.sim_params["noise_amplitude"] = float(self.noise_amp_var.get())
                except ValueError:
                    pass
                
                self.sim_params["outlier_enabled"] = self.outlier_enabled_var.get()
                try:
                    self.sim_params["outlier_amplitude"] = float(self.outlier_amp_var.get())
                except ValueError:
                    pass
                try:
                    self.sim_params["outlier_max_duration"] = float(self.outlier_dur_var.get()) / 1000.0
                except ValueError:
                    pass
                try:
                    self.sim_params["outlier_frequency"] = float(self.outlier_freq_var.get())
                except ValueError:
                    pass
                
                try:
                    period_ms = float(self.send_period_var.get())
                    self.sim_params["send_period"] = max(period_ms, 10) / 1000.0
                except ValueError:
                    pass
        except Exception as e:
            print(f"Ошибка обновления параметров: {e}")
        
        self.root.after(100, self.update_params_from_gui)
    
    def simulation_loop(self):
        """Основной цикл симуляции системы"""
        last_send_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                dt = 0.01
                
                with self.lock:
                    params = self.sim_params.copy()
                
                self.pid.kp = params["kp"]
                self.pid.ki = params["ki"]
                self.pid.kd = params["kd"]
                
                with self.lock:
                    correction = self.pid.update(self.setpoint, self.current_value, dt)
                    self.current_value += correction * dt
                
                noisy_value = self.current_value
                
                if params["noise_enabled"]:
                    if params["noise_type"] == "gaussian":
                        noise = random.gauss(0, params["noise_amplitude"])
                    else:
                        noise = random.uniform(-params["noise_amplitude"], params["noise_amplitude"])
                    noisy_value += noise
                
                if params["outlier_enabled"]:
                    if not self.active_outlier["active"]:
                        if random.random() < (params["outlier_frequency"] * dt):
                            self.active_outlier = {
                                "active": True,
                                "remaining_time": random.uniform(0.001, params["outlier_max_duration"]),
                                "value": random.uniform(-params["outlier_amplitude"], params["outlier_amplitude"]) * random.choice([-1, 1])
                            }
                            if self.debug_outliers:
                                self.add_log_message(f"Выброс: {self.active_outlier['value']:.2f}, длит: {self.active_outlier['remaining_time']*1000:.1f} мс")
                    
                    if self.active_outlier["active"]:
                        noisy_value += self.active_outlier["value"]
                        self.active_outlier["remaining_time"] -= dt
                        
                        if self.active_outlier["remaining_time"] <= 0:
                            self.active_outlier["active"] = False
                
                with self.lock:
                    self.noisy_value = noisy_value
                
                if current_time - last_send_time >= params["send_period"]:
                    ts = int((current_time - self.start_time) * 1000)
                    packet = struct.pack("<If", ts, noisy_value)
                    
                    try:
                        if self.sock:
                            send_ip = self.send_ip_var.get()
                            send_port = int(self.send_port_var.get())
                            self.sock.sendto(packet, (send_ip, send_port))
                    except Exception as e:
                        if "10054" not in str(e):
                            self.add_log_message(f"Ошибка отправки: {e}")
                    
                    last_send_time = current_time
                
                time.sleep(dt)
                
            except Exception as e:
                self.add_log_message(f"Ошибка в simulation_loop: {e}")
                time.sleep(0.1)
    
    def receive_loop(self):
        """Цикл приема данных по UDP"""
        while self.running:
            try:
                if self.sock:
                    data, addr = self.sock.recvfrom(1024)
                    if len(data) == 4:
                        new_setpoint = struct.unpack("<f", data)[0]
                        with self.lock:
                            self.setpoint = new_setpoint
                        self.add_log_message(f"Принята Целевое значение: {new_setpoint:.2f} от {addr}")
            except BlockingIOError:
                time.sleep(0.01)
            except ConnectionResetError:
                pass
            except OSError as e:
                if e.winerror == 10054:
                    pass
                elif self.running:
                    self.add_log_message(f"Ошибка приема: {e}")
            except Exception as e:
                if self.running:
                    self.add_log_message(f"Ошибка приема: {e}")
    
    def apply_network_settings(self):
        """Применение новых сетевых настроек"""
        try:
            self.send_ip = self.send_ip_var.get()
            self.send_port = int(self.send_port_var.get())
            
            self.receive_ip = self.receive_ip_var.get()
            self.receive_port = int(self.receive_port_var.get())
            
            self.init_network()
            
            self.add_log_message("Сетевые настройки применены")
            
        except Exception as e:
            self.add_log_message(f"Ошибка применения сетевых настроек: {e}")
    
    def update_display(self):
        """Обновление цифрового отображения"""
        with self.lock:
            noisy_val = self.noisy_value
            setpoint_val = self.setpoint
        
        self.current_value_label.config(text=f"{noisy_val:.3f}")
        self.setpoint_label.config(text=f"{setpoint_val:.3f}")
        
        status = f"Текущее (c шумами): {noisy_val:.3f} | Целевое значение: {setpoint_val:.3f}"
        if self.active_outlier["active"]:
            status += f" | Выброс активен: {self.active_outlier['value']:.1f}"
        self.status_bar.config(text=status)
        
        self.root.after(50, self.update_display)
    
    def add_log_message(self, message):
        """Добавление сообщения в лог через очередь"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        full_message = f"[{timestamp}] {message}"
        self.log_queue.put(full_message)
    
    def process_log_queue(self):
        """Обработка очереди логов из главного потока"""
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.log.insert(tk.END, message + "\n")
                self.log.see(tk.END)
                
                lines = self.log.get(1.0, tk.END).split('\n')
                if len(lines) > 100:
                    self.log.delete(1.0, 2.0)
        except queue.Empty:
            pass
        except:
            pass
        
        self.root.after(50, self.process_log_queue)
    
    def on_closing(self):
        """Обработка закрытия приложения"""
        self.running = False
        
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
        
        self.root.destroy()

        import os
        os._exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    
    root.geometry("1000x900")
    root.minsize(900, 900)
    app = DynamicSystemApp(root)
    root.mainloop()