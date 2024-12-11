import sys
import socket
from PyQt5.QtCore import QThread, pyqtSignal

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QPushButton, QLCDNumber, QLineEdit
)
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas
)
from matplotlib.figure import Figure

import sqlite3
from datetime import datetime
import time
import numpy as np




class TcpWorker(QThread): #El encargado de manejar la conexion TCP, recibir y enviar datos
    data_received = pyqtSignal(str)
    client_connected = pyqtSignal(str)

    def __init__(self, host='0.0.0.0', port=1234):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen()
        self.conn = None  # Variable que almacena la conexion

    def run(self):
        print(f"Server listening on {self.port}")
        while self.running:
            conn, addr = self.server.accept()
            self.conn = conn  # Genera la conexión y la guarda de forma global para usarla en otras instancias
            self.client_connected.emit(f"Connected by {addr}")
            with conn:
                data = conn.recv(512)
                if data:
                    message = data.decode('utf-8')
                    self.data_received.emit(message)
                    response = f"Received: {message} \0"
                    #conn.sendall(response.encode('utf-8'))
    
    def send_message(self, message): #Manda mensajes a la ESP32
        if self.conn:
            try:
                self.conn.sendall(f"{message}\0".encode('utf-8'))
                print(f"Sent: {message}")
            except Exception as e:
                print(f"Error sending message: {e}")
        else:
            print("No client connected to send the message.")
                
    def stop(self): #Mata el puerto cuando se corta el programa
        self.running = False
        if self.conn:
            self.conn.close()
        self.server.close()


        


        
class MyApp(QMainWindow): #Es la aplicacion principal, donde se corren la interfaz, la base de datos y la conexion TCP
    def __init__(self):
        super().__init__()
        self.start_time = time.time() #Tiempo inicial para los graficos
        self.initDatabase()
        self.initUI() #Corre la interfaz
        self.initTCP() #Corre la conexion TCP
        self.init_db() #Corre la base de datos

    def initUI(self):
        self.setWindowTitle('Interfaz del Controlador')
        self.setGeometry(100, 100, 1200, 600)

        # Genera las partesde la interfaz
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()
        right_layout2 = QVBoxLayout()

        # Graficos de la izquierda
        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        left_layout.addWidget(self.canvas)


        # Elementos de la derecha (botones y valores instantaneos)
        self.temp_display = QLCDNumber()
        self.temp_display.setDigitCount(5)
        self.temp_display.setMode(QLCDNumber.Dec)
        
        self.temp_displayM = QLCDNumber()
        self.temp_displayM.setDigitCount(5)
        self.temp_displayM.setMode(QLCDNumber.Dec)
        
        self.temp_displayP = QLCDNumber()
        self.temp_displayP.setDigitCount(5)
        self.temp_displayP.setMode(QLCDNumber.Dec)

        self.hum_display = QLCDNumber()
        self.hum_display.setDigitCount(5)
        self.hum_display.setMode(QLCDNumber.Dec)
        
        self.hum_displayM = QLCDNumber()
        self.hum_displayM.setDigitCount(5)
        self.hum_displayM.setMode(QLCDNumber.Dec)
        
        self.hum_displayP = QLCDNumber()
        self.hum_displayP.setDigitCount(5)
        self.hum_displayP.setMode(QLCDNumber.Dec)

        self.btn_start = QPushButton("Comenzar envio de datos")
        self.btn_stop = QPushButton("Parar envio de datos")
        self.btn_prom = QPushButton("Enviar de promedio de datos")
        self.btn_inst = QPushButton("Enviar datos instantaneos")
        self.btn_freq = QPushButton("Cambiar frecuencia de muestreo")

        self.freq_input = QLineEdit()
        self.freq_input.setPlaceholderText("Ingresar frecuencia:")
        self.freq_input.setAlignment(Qt.AlignCenter)

        self.labelT = QLabel("Temperatura (Celsius)")
        font = QFont()
        font.setPointSize(16)  # Set the font size
        self.labelT.setFont(font)
        self.labelH= QLabel("Humedad (Porcentual)")
        self.labelH.setFont(font)
        
        
        right_layout.addWidget(self.btn_start)
        right_layout.addWidget(self.btn_stop)
        right_layout.addWidget(self.btn_prom)
        right_layout.addWidget(self.btn_inst)
        right_layout.addWidget(self.btn_freq)
        right_layout.addWidget(self.freq_input)



        right_layout2.addWidget(self.labelT)
        right_layout2.addWidget(QLabel("ultimo valor medido"))
        right_layout2.addWidget(self.temp_display)
        right_layout2.addWidget(QLabel("Maximo Registrado"))
        right_layout2.addWidget(self.temp_displayM)
        right_layout2.addWidget(QLabel("Promedio de las mediciones"))
        right_layout2.addWidget(self.temp_displayP)


        right_layout2.addWidget(self.labelH)
        right_layout2.addWidget(QLabel("ultimo valor medido"))
        right_layout2.addWidget(self.hum_display)
        right_layout2.addWidget(QLabel("Maximo Registrado"))
        right_layout2.addWidget(self.hum_displayM)
        right_layout2.addWidget(QLabel("Promedio de las mediciones"))
        right_layout2.addWidget(self.hum_displayP)
        

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout2)
        main_layout.addLayout(right_layout)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Conecta los botones:
        self.btn_start.clicked.connect(self.handle_start)
        self.btn_stop.clicked.connect(self.handle_stop)
        self.btn_prom.clicked.connect(self.handle_prom)
        self.btn_inst.clicked.connect(self.handle_inst)
        self.btn_freq.clicked.connect(self.handle_change_freq)



    def initTCP(self): #Inicializa la comunicacion TCP llamando a TcpWorker
        self.tcp_thread = TcpWorker()
        self.tcp_thread.data_received.connect(self.display_data)
        self.tcp_thread.client_connected.connect(self.show_connection_status)
        self.tcp_thread.start()





    def display_data(self, data): #La encargada de almacenar los valores en la base de datos y mostrarlos en la interfaz como valores instantanteos
        print("Received:", data)
        try:
            # Recibe los valores desde la ESP32
            parts = data.split()
            temp = float(parts[1])
            hum = float(parts[3])

            # Muestra los valores en la interfaz
            self.temp_display.display(temp)
            self.hum_display.display(hum)

            #Almacena la informacion extraida en la base de datos
            current_time = time.time()
            time_diff = current_time - self.start_time #Almacena cuanto tiempo ha transcurrido desde que se inicio el programa
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.db_cursor.execute('''
                INSERT INTO data (timestamp, temperature, humidity)
                VALUES (?, ?, ?)
            ''', (time_diff, temp, hum))
            self.db_conn.commit()

            # Actualiza el grafico en la interfaz
            self.plot_data()

        except (IndexError, ValueError):
            print("Invalid data format")



    def initDatabase(self): 
        #Reinicia la base de datos, se corre al iniciar el programa
        conn = sqlite3.connect('sensor_data.db')
        c = conn.cursor()
        c.execute('DELETE FROM data')
        conn.commit()
        conn.close()



    def show_connection_status(self, status):
        print(status) #Muestra el estado de conexion en la consola


    #Seccion de los botones:

    def handle_start(self):
        print("Se activo el envio de datos")
        self.tcp_thread.send_message("si\0")
            
    def handle_stop(self):
        print("Se desactivo el envio de datos")
        self.tcp_thread.send_message("no\0")

    def handle_prom(self):
        print("Se cambio a modo promedio")
        self.tcp_thread.send_message("prom\0")
            
    def handle_inst(self):
        print("Se cambio a modo instantaneo")
        self.tcp_thread.send_message("inst\0")


    def handle_change_freq(self):
        freq = self.freq_input.text()
        if freq in ["100", "500", "1000"]:
            print(f"Se cambio la frecuencia de muestreo a: {freq}")
            self.tcp_thread.send_message(f"{freq}" + "\0")
        else:
            print("Frecuencia Invalida")

    
    
    
    #Inicializa la base de datos:
    def init_db(self):
        self.db_conn = sqlite3.connect("sensor_data.db")
        self.db_cursor = self.db_conn.cursor()
        
        #Crea la tabla si no existe
        self.db_cursor.execute('''
            CREATE TABLE IF NOT EXISTS data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                temperature REAL,
                humidity REAL
            )
        ''')
        self.db_conn.commit()

    #Permite la extracción de información de la base de datos:
    def extract_data_from_db(self):
        self.db_cursor.execute('''
            SELECT timestamp, temperature, humidity FROM data
        ''')

        # Saca toda la información de la base de datos:
        results = self.db_cursor.fetchall()

        # Guarda los datos extraidos en vectores de tiempo, temperatura y humedad
        timestamps = [row[0] for row in results]
        temperatures = [row[1] for row in results]
        humidities = [row[2] for row in results]

        return timestamps, temperatures, humidities


            
        
    def plot_data(self):
        # extrae la data:
        timestamps, temperatures, humidities = self.extract_data_from_db()
        
        
        #La transforma de strings a floats:
        timestamps = list(map(float,timestamps))
        temperatures = list(map(float,temperatures))
        humidities = list(map(float,humidities))
        
        #Valores maximos registrados:
        temM = np.max(temperatures)
        humM = np.max( humidities)
        
        #Valores promedio registrados:
        humP = np.mean(humidities)
        temP = np.mean(temperatures)
        
        
        self.temp_displayP.display(temP)
        self.hum_displayP.display(humP)
        
        self.temp_displayM.display(temM)
        self.hum_displayM.display(humM)

        if not timestamps:
            print("No data to plot.")
            return

        # Limpia la figura del ciclo anterior
        self.figure.clear()


        # Grafico temperatura:
        ax1 = self.figure.add_subplot(211)  
        ax1.plot( timestamps, temperatures, label="Temperatura (°C)", color='red')
        ax1.set_title("Temperatura y Humendad en el tiempo")
        ax1.set_xlabel("tiempo (s)")
        ax1.set_ylabel("Temperatura (°C)")
        ax1.legend()

        # Grafico humedad:
        ax2 = self.figure.add_subplot(212)  # 2 rows, 1 column, second subplot
        ax2.plot( timestamps, humidities, label="Humedad (%)", color='blue')
        #ax2.set_title(" ")
        ax2.set_xlabel("tiempo (s)")
        ax2.set_ylabel("Humedad (%)")
        ax2.legend()


        

        # dibuja el grafico:
        self.canvas.draw()


    #Cierra todos los procesos cuando se apaga el codigo:
    def closeEvent(self, event):
        self.tcp_thread.stop()
        self.db_conn.close()
        event.accept()

#Ciclo principal donde se llama a MyApp() 
if __name__ == '__main__':
    app = QApplication(sys.argv)
    my_app = MyApp()
    my_app.show()
    sys.exit(app.exec_())
    
    