import socket

HOST = '0.0.0.0'  # Escucha en todas las interfaces disponibles
PORT = 1234       # Puerto en el que se escucha

# Crea un socket para IPv4 y conexión TCP
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    print("El servidor está esperando conexiones en el puerto", PORT)

    while True:
        conn, addr = s.accept()  # espera una conexión
        with conn:
            print('Conectado por', addr)

            #freciencia de muestreo
            frecuencia = input("Introduce la frecuencia de muestreo (100, 500, 1000): ")
            if frecuencia in ["100", "500", "1000"]:
                conn.sendall(frecuencia.encode('utf-8'))  # Envía la frecuencia al cliente ESP32
                print(f"Frecuencia {frecuencia} enviada al cliente.")
            else:
                print("Frecuencia no válida. Debe ser 100, 500 o 1000.")