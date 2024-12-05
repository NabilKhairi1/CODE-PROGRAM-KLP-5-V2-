import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
from queue import Queue

class PengendaliMotorDC:
    def __init__(self, root):
        self.root = root
        self.root.title("Pengendali Motor DC PID")
        self.root.geometry("1500x800")
        self.root.configure(bg="#B0C4DE")

        # Variabel
        self.is_terhubung = False
        self.arduino = None
        self.antrian_data = Queue()
        self.sedang_berjalan = False
        self.arah_maju = True  # Tambahkan variabel arah motor
        self.data_terakhir = {'rpm': 0, 'error': 0, 'pwm': 0}

        # Data Plot
        self.data_waktu = []
        self.data_rpm = []
        self.data_error = []
        self.data_pwm = []
        self.data_setpoint = []
        self.waktu_mulai = time.time()

        # Setup GUI
        self.setup_gui()

        # Setup Plot
        self.setup_plot()

        # Temukan port Arduino yang tersedia
        self.perbarui_port()

    def setup_gui(self):
        # Panel Kontrol
        frame_kontrol = ttk.LabelFrame(self.root, text="Panel Kontrol", padding="10")
        frame_kontrol.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        frame_kontrol.configure(style="Custom.TLabelframe")

        ttk.Label(frame_kontrol, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.var_port = tk.StringVar()
        self.combo_port = ttk.Combobox(frame_kontrol, textvariable=self.var_port)
        self.combo_port.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(frame_kontrol, text="Refresh", command=self.perbarui_port).grid(row=0, column=2, padx=5, pady=5)
        self.tombol_sambung = ttk.Button(frame_kontrol, text="Connect", command=self.toggle_sambungan)
        self.tombol_sambung.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Kp:").grid(row=1, column=0, padx=5, pady=5)
        self.var_kp = tk.DoubleVar(value=1.0)
        self.entry_kp = ttk.Entry(frame_kontrol, textvariable=self.var_kp)
        self.entry_kp.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Ki:").grid(row=2, column=0, padx=5, pady=5)
        self.var_ki = tk.DoubleVar(value=0.5)
        self.entry_ki = ttk.Entry(frame_kontrol, textvariable=self.var_ki)
        self.entry_ki.grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Kd:").grid(row=3, column=0, padx=5, pady=5)
        self.var_kd = tk.DoubleVar(value=0.1)
        self.entry_kd = ttk.Entry(frame_kontrol, textvariable=self.var_kd)
        self.entry_kd.grid(row=3, column=1, padx=5, pady=5)

        ttk.Button(frame_kontrol, text="Perbarui PID", command=self.perbarui_pid).grid(row=4, column=0, columnspan=3, pady=10)

        ttk.Label(frame_kontrol, text="Setpoint (RPM):").grid(row=5, column=0, padx=5, pady=5)
        self.var_setpoint = tk.IntVar(value=300)
        self.entry_setpoint = ttk.Entry(frame_kontrol, textvariable=self.var_setpoint)
        self.entry_setpoint.grid(row=5, column=1, padx=5, pady=5)

        # Tombol kontrol motor
        ttk.Button(frame_kontrol, text="Run", command=self.jalankan_motor_searah).grid(row=6, column=0, padx=1, pady=5)
        ttk.Button(frame_kontrol, text="Stop", command=self.berhentikan_motor).grid(row=6, column=1, padx=1, pady=5)
        
        # Tambahkan tombol arah
        self.tombol_arah = ttk.Button(frame_kontrol, text="Turn Around", command=self.ubah_arah)
        self.tombol_arah.grid(row=6, column=2, padx=1, pady=5)

        self.label_rpm = ttk.Label(frame_kontrol, text="RPM Aktual: 0")
        self.label_rpm.grid(row=7, column=0, columnspan=3, pady=10)

        self.label_status = ttk.Label(frame_kontrol, text="Status: Tidak Terhubung")
        self.label_status.grid(row=8, column=0, columnspan=3, pady=10)

        self.style = ttk.Style()
        self.style.configure("Custom.TLabelframe", background="#B0C4DE")
        self.style.configure("Custom.TLabelframe.Label", background="#B0C4DE", foreground="black")
        self.style.configure("TLabel", background="#B0C4DE", foreground="black")

    def setup_plot(self):
        frame_plot = ttk.LabelFrame(self.root, text="Plot Grafik", padding="10")
        frame_plot.grid(row=0, column=1, padx=10, pady=5, sticky="nsew")
        frame_plot.configure(style="Custom.TLabelframe")

        self.fig = Figure(figsize=(10.2, 7.45), dpi=100)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        
        self.ax1.set_title('Respon RPM Motor')
        self.ax1.set_xlabel('Waktu (s)')
        self.ax1.set_ylabel('RPM')
        self.ax1.grid(True)

        self.ax2.set_title('PWM dan Error')
        self.ax2.set_xlabel('Waktu (s)')
        self.ax2.set_ylabel('PWM/Error')
        self.ax2.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_plot)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.garis_rpm, = self.ax1.plot([], [], 'b-', label='RPM Aktual')
        self.garis_setpoint, = self.ax1.plot([], [], 'r--', label='Setpoint')
        
        self.garis_pwm, = self.ax2.plot([], [], 'g-', label='PWM')
        self.garis_error, = self.ax2.plot([], [], 'r--', label='Error')
        
        self.ax1.legend(loc='upper left')
        self.ax2.legend(loc='upper left')

        self.fig.tight_layout()

    def perbarui_port(self):
        try:
            port = [p.device for p in serial.tools.list_ports.comports()]
            self.combo_port['values'] = port
            if port:
                self.combo_port.set(port[0])
        except Exception as e:
            messagebox.showerror("Kesalahan", f"Gagal memperbarui port: {e}")

    def toggle_sambungan(self):
        if not self.is_terhubung:
            try:
                port = self.var_port.get()
                # Tambahkan timeout dan eksplisit error handling
                self.arduino = serial.Serial(port, 115200, timeout=2)
                time.sleep(2)  # Tunggu inisialisasi Arduino
                self.is_terhubung = True
                self.tombol_sambung.config(text="Disconnect")
                self.label_status.config(text="Status: Terhubung")

                # Reset data
                self.data_waktu = []
                self.data_rpm = []
                self.data_error = []
                self.data_pwm = []
                self.waktu_mulai = time.time()

                self.sedang_berjalan = True
                threading.Thread(target=self.baca_serial, daemon=True).start()
                threading.Thread(target=self.perbarui_plot, daemon=True).start()

            except Exception as e:
                messagebox.showerror("Kesalahan Sambungan", str(e))
                self.label_status.config(text=f"Kesalahan: {str(e)}")
        else:
            self.sedang_berjalan = False
            if self.arduino:
                try:
                    self.arduino.close()
                except:
                    pass
            self.is_terhubung = False
            self.tombol_sambung.config(text="Connect")
            self.label_status.config(text="Status: Tidak Terhubung")

    def perbarui_pid(self):
        if self.is_terhubung:
            try:
                kp = self.var_kp.get()
                ki = self.var_ki.get()
                kd = self.var_kd.get()
                setpoint = self.var_setpoint.get()
                
                # Kirim parameter dalam satu perintah
                perintah = f"{kp},{ki},{kd},{setpoint}\n"
                self.arduino.write(perintah.encode('utf-8'))
            except Exception as e:
                messagebox.showerror("Kesalahan", f"Gagal memperbarui PID: {e}")

    def jalankan_motor_searah(self):
        if self.is_terhubung:
            try:
                self.arduino.write(b"START\n")
            except Exception as e:
                messagebox.showerror("Kesalahan", f"Gagal menjalankan motor: {e}")

    def berhentikan_motor(self):
        if self.is_terhubung:
            try:
                self.arduino.write(b"STOP\n")
            except Exception as e:
                messagebox.showerror("Kesalahan", f"Gagal menghentikan motor: {e}")

    def ubah_arah(self):
        if self.is_terhubung:
            try:
                if self.arah_maju:
                    self.arduino.write(b"REVERSE\n")
                    self.tombol_arah.config(text="Turn Around")
                    self.arah_maju = False
                else:
                    self.arduino.write(b"FORWARD\n")
                    self.tombol_arah.config(text="Turn Around")
                    self.arah_maju = True
            except Exception as e:
                messagebox.showerror("Kesalahan", f"Gagal mengubah arah: {e}")

    def baca_serial(self):
        while self.sedang_berjalan and self.is_terhubung:
            try:
                # Tambahkan error handling yang lebih baik
                baris = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if baris.count(',') == 2:
                    try:
                        rpm, error, pwm = map(float, baris.split(','))
                        waktu_saat_ini = time.time() - self.waktu_mulai
                        self.antrian_data.put((waktu_saat_ini, rpm, error, pwm))
                    except ValueError:
                        continue
            except Exception as e:
                self.label_status.config(text=f"Kesalahan Baca Serial: {str(e)}")
                break

    def perbarui_plot(self):
        while self.sedang_berjalan:
            try:
                while not self.antrian_data.empty():
                    waktu_saat_ini, rpm, error, pwm = self.antrian_data.get()
                    # Simpan data terakhir
                    self.data_terakhir = {'rpm': rpm, 'error': error, 'pwm': pwm}
                    
                    self.data_waktu.append(waktu_saat_ini)
                    self.data_rpm.append(rpm)
                    self.data_error.append(error)
                    self.data_pwm.append(pwm)

                    # Simpan hanya 100 titik data terakhir
                    if len(self.data_waktu) > 100:
                        self.data_waktu = self.data_waktu[-100:]
                        self.data_rpm = self.data_rpm[-100:]
                        self.data_error = self.data_error[-100:]
                        self.data_pwm = self.data_pwm[-100:]

                    # Update plot RPM
                    self.ax1.set_xlim(max(0, self.data_waktu[-1] - 10), self.data_waktu[-1])
                    self.garis_rpm.set_data(self.data_waktu, self.data_rpm)
                    self.garis_setpoint.set_data(self.data_waktu, [self.var_setpoint.get()] * len(self.data_waktu))

                    # Update plot PWM dan Error
                    self.ax2.set_xlim(max(0, self.data_waktu[-1] - 10), self.data_waktu[-1])
                    self.garis_pwm.set_data(self.data_waktu, self.data_pwm)
                    self.garis_error.set_data(self.data_waktu, self.data_error)

                    # Adjust y-limits dynamically
                    if self.data_rpm:
                        self.ax1.set_ylim(0, max(self.data_rpm) * 1.2)
                    if self.data_pwm:
                        self.ax2.set_ylim(min(self.data_pwm + self.data_error), max(self.data_pwm + self.data_error) * 1.2)

                    self.canvas.draw()
                    # Perbarui label RPM dari data terakhir
                    self.label_rpm.config(text=f"RPM Aktual: {int(rpm)}")

                time.sleep(0.05)
            except Exception as e:
                self.label_status.config(text=f"Kesalahan Plot: {str(e)}")
                break

if __name__ == "__main__":
    root = tk.Tk()
    app = PengendaliMotorDC(root)
    root.mainloop()