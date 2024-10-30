import socket

HOST = '0.0.0.0'  # Escucha en todas las interfaces disponibles
PORT = 1234       # Puerto en el que se escucha

ConInFrec = True
InConex = True

# Crea un socket para IPv4 y conexión TCP
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()


    print("El servidor esta esperando conexiones en el puerto", PORT)
    

    while InConex:  #Corresponde al envio inicial de data
        conn, addr = s.accept()  # espera una conexión
        with conn:
            print('Conectado por', addr)

            #freciencia de muestreo
            Conexion = input("Quiere comenzar la transmición de datos? (Si/No): ")
            if Conexion in ["Si","si","SI"]:
                Conexion = "si\0"
                conn.sendall(Conexion.encode('utf-8'))  # Envía la frecuencia al cliente ESP32
                print(f"Se Comenzara la transmición de datos")
                InConex = False
            else:
                print("Este codigo no sabe que hacer si no inicializan la transmición")


    

    while ConInFrec:  #Corresponde al envio inicial de data
        conn, addr = s.accept()  # espera una conexión
        with conn:
            print('Conectado por', addr)

            #freciencia de muestreo
            frecuencia = input("Introduce la frecuencia de muestreo (100, 500, 1000): ")
            if frecuencia in ["100", "500", "1000"]:
                frecuencia = frecuencia + "\0"
                conn.sendall(frecuencia.encode('utf-8'))  # Envía la frecuencia al cliente ESP32
                print(f"Frecuencia {frecuencia} enviada al cliente.")
                ConInFrec = False
            else:
                print("Frecuencia no válida. Debe ser 100, 500 o 1000.")



    while True:
        conn, addr = s.accept()  # Espera una conexión
        with conn:
            print('Conectado por', addr)
            data = conn.recv(512)  # Recibe hasta 1024 bytes del cliente
            if data:
                print("Recibido: ", data.decode('utf-8'))
                respuesta = "tu mensaje es: " + data.decode('utf-8')
                conn.sendall(respuesta.encode('utf-8'))  # Envia la respuesta al cliente
