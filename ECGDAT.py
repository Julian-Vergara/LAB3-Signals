import sys
from PyQt6 import uic, QtCore, QtWidgets
from PyQt6.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QFileDialog
from PyQt6.QtCore import *
from PyQt6.QtGui import *

import serial.tools.list_ports
import serial
import numpy as np
import struct
from scipy import stats

import threading

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

import datetime
import numpy.fft as fft

class principal(QMainWindow):

    # Creación del hilo para trabajo de adquisición de datos para trabajo en tiempo real
    def __init__(self):
        super(principal, self).__init__()
        uic.loadUi("ECGD.ui", self)
        self.puertos_disponibles()
        
        self.ser1 = None
        
        self.connect.clicked.connect(self.conectar)
        self.guardarButton.clicked.connect(self.guardar_datos)
        self.cargarButton.clicked.connect(self.cargar_y_mostrar_datos)

        self.x = np.linspace(0, 10, 6000)
        self.y = np.zeros(6000)  

        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.graficawidget.setLayout(layout)

        self.fm = 1000  # Frecuencia de muestreo

        # Frecuencias de corte
        self.fc_baja = 20  # Pasa altos
        self.fc_alta = 450  # Pasa bajos

        # Normalización de frecuencias de corte
        self.fn_baja = self.fc_baja / (0.5 * self.fm)
        self.fn_alta = self.fc_alta / (0.5 * self.fm)

        # Orden del filtro
        self.orden = 4

        # Filtro pasa banda
        self.b, self.a = butter(self.orden, [self.fn_baja, self.fn_alta], btype='bandpass', analog=False)

    # Determinación de los puertos para la detección de datos
    def puertos_disponibles(self):
        p = serial.tools.list_ports.comports()
        for port in p:
            self.puertos.addItem(port.device)

    # Conexion con el puerto
    def conectar(self): 
        estado = self.connect.text()
        self.stop_event_ser1 = threading.Event()
        if estado == "CONECTAR":
            com = self.puertos.currentText()
            try:
                self.ser1 = serial.Serial(com, 115200)
                self.hilo_ser1 = threading.Thread(target=self.periodic_thread1)
                self.hilo_ser1.start()
                print("Puerto serial 1 Conectado")
                self.connect.setText("DESCONECTAR")

            except serial.SerialException as e:
                print("Error en el puerto serial 1: ", e)
        else:
            self.ser1.close()
            self.stop_event_ser1.set()
            self.hilo_ser1.join()
            print("Puerto serial 1 Desconectado")
            self.connect.setText("CONECTAR")

    # Desempaquetado de datos de 50B
    def periodic_thread1(self):
        if self.ser1 is not None and self.ser1.is_open:
            data = self.ser1.read(50)
            if len(data) == 50:
                data = struct.unpack('50B', data)
                for i in range(0, len(data), 2):
                    self.y = np.roll(self.y, -1)
                    self.y[-1] = data[i] * 100 + data[i + 1]

                self.ax.clear()

                # Aplicar el filtro pasa banda
                df = filtfilt(self.b, self.a, self.y)

                # Graficar la señal filtrada
                self.ax.plot(self.x, df)
                self.ax.grid(True)
                self.canvas.draw()

        if not self.stop_event_ser1.is_set():
            threading.Timer(1e-3, self.periodic_thread1).start()

    def guardar_datos(self):
        try:
            now = datetime.datetime.now()
            fecha_hora = now.strftime("%Y-%m-%d %H:%M:%S")
            nombre_persona = self.nombre_persona.text()
            nombre_persona = nombre_persona.replace(":", "").replace(" ", "_")
            nombre_archivo = f"{nombre_persona}.txt"

            # Aplicar el filtro a la señal actual antes de guardarla
            df = filtfilt(self.b, self.a, self.y)

            # Guardar la señal filtrada en el archivo en .txt
            with open(nombre_archivo, 'w') as f:
                f.write(f"Fecha y hora: {fecha_hora}\n")
                f.write(f"Nombre del paciente: {nombre_persona}\n")
                f.write("Datos filtrados de la medición:\n")
                for i in range(len(self.x)):
                    f.write(f"{self.x[i]}, {df[i]}\n")

            print(f"Datos filtrados guardados en {nombre_archivo}")

        except Exception as e:
            print("Error al guardar los datos:", e)
            
    #Funcion principal para leer los datos del txt
    def cargar_datos(self, nombre_archivo):
        try:
            x = []
            y = []

            with open(nombre_archivo, 'r') as f:
                for _ in range(4):
                    next(f)

                for line in f:
                    datos = line.strip().split(",")
                    x.append(float(datos[0]))
                    y.append(float(datos[1]))
            return x, y

        except Exception as e:
            print("Error al cargar los datos:", e)
            return None, None
        
    #Deteccion de la actividad muscular
    def detectar_pulsos(self, señal, umbral):
        pulsos = []
        en_pulso = False
        inicio_pulso = 0

        for i in range(len(señal)):
            if señal[i] > umbral and not en_pulso:
                en_pulso = True
                inicio_pulso = i
            elif señal[i] < umbral and en_pulso:
                en_pulso = False
                fin_pulso = i
                pulsos.append((inicio_pulso, fin_pulso))
        
        return pulsos

    def aplicar_ventana_hanning_por_pulsos(self, y, pulsos):
        
        señal_filtrada = np.zeros_like(y)
        
        for inicio, fin in pulsos:
            ventana_hanning = np.hanning(fin - inicio)
            señal_filtrada[inicio:fin] = y[inicio:fin] * ventana_hanning
        
        return señal_filtrada

    def calcular_y_mostrar_fft(self, y_filtrada):
        # Calcular la Transformada de Fourier de la señal aventanada
        Y_fft = fft.fft(y_filtrada)
        
        # Crear el vector de frecuencias correspondiente
        N = len(y_filtrada)
        freqs = fft.fftfreq(N, 1 / self.fm)
        
        # Eliminar la parte negativa de la ventna
        Y_fft_magnitude = np.abs(Y_fft[:N // 2])
        freqs_pos = freqs[:N // 2]

        # Pasar a decibelios
        Y_fft_magnitude_db = 20 * np.log10(Y_fft_magnitude + 1e-10)

        
        # Graficar espectro
        fig_fft, ax_fft = plt.subplots()

        
        ax_fft.plot(freqs_pos, Y_fft_magnitude_db)
        ax_fft.set_xlabel('Frecuencia (Hz)')
        ax_fft.set_ylabel('Magnitud (dB)')
        ax_fft.set_title('Espectro de Frecuencias')
        ax_fft.grid(True)

        plt.show()


    def cargar_y_mostrar_datos(self):
        nombre_archivo, _ = QFileDialog.getOpenFileName(self, "Seleccionar archivo", "", "Archivos de texto (*.txt)")
        if nombre_archivo:
            x, y = self.cargar_datos(nombre_archivo)
            if x and y:
                self.x = x
                self.y = y

                # Pulsos
                umbral = 20 
                pulsos = self.detectar_pulsos(self.y, umbral)

                # Aplicar la ventana 
                y_filtrada = self.aplicar_ventana_hanning_por_pulsos(self.y, pulsos)

                # Limpiar la gráfica para eliminar los datos capturados 
                self.ax.clear()

                # Graficar la señal original
                self.ax.plot(self.x, self.y, label="Original")

                # Graficar las ventanas
                self.ax.plot(self.x, y_filtrada, label="Ventanas Hanning", linestyle="--")

                self.ax.set_xlabel('Tiempo')
                self.ax.set_ylabel('Valor')
                self.ax.set_title('Datos con Ventana de Hanning en Pulsos')
                self.ax.legend()

                # Refrescar el canvas para mostrar la gráfica
                self.canvas.draw()

                # Calcular y mostrar la Transformada de Fourier de la señal aventanada
                self.calcular_y_mostrar_fft(y_filtrada)

                # Calcular las medianas de los pulsos
                medianas = []
                for inicio, fin in pulsos:
                    pulso = y_filtrada[inicio:fin]
                    if len(pulso) > 0:  # Asegurarse de que hay datos en el pulso
                        medianas.append(np.median(pulso))

                # Obtener la última mediana
                if len(medianas) > 0:
                    ultima_mediana = medianas[-1]
                    # Calcular la media de las medianas
                    media_medianas = np.mean(medianas[:-1]) if len(medianas) > 1 else medianas[0]

                    # Aplicar la prueba de Wilcoxon
                    stat, p_value = stats.wilcoxon(medianas[:-1], [ultima_mediana]*len(medianas[:-1]))

                    # Mostrar resultados
                    print(f"Mediana de todos los pulsos: {media_medianas}")
                    print(f"Última mediana: {ultima_mediana}")
                    print(f"Estadístico de Wilcoxon: {stat}, p-valor: {p_value}")

                    # Interpretar el p-valor
                    alpha = 0.05
                    if p_value < alpha:
                        print("El cambio en la mediana es significativo.")
                    else:
                        print("El cambio en la mediana no es significativo.")
                else:
                    print("No se encontraron pulsos en la señal.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    GUI = principal()
    GUI.show()
    sys.exit(app.exec())


