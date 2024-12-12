### Integrantes:
- Tomás Troncoso
- Javiera Mora 

<h1>Descripción proyecto Final (Tarea 2)</h1>

## Descripción general del código en c
Para el funcionamiento del código en c se debe ingresar al [código del ESP32](https://github.com/proyectostic3/TareasG4Tic3/tree/main/Tarea%202/TIC3ProyectoFinal/main/ProjectoFinal_TIC3.c), el cual hace que este último realice la conexión TCP a través de una red Wi-Fi con el servidor que se encuentra en la raspberry py, en esta conexión se envían los datos de temperatura y humedad al servidor especificado. El LED parpadea durante el envío de datos (cuando ocurre la conexción TCP).

Esta comunicación es capaz de iniciarse o detenerse respecto al envio de datos, además de que se puede cambiar frecuencia de muestreo ( de 10, 2, 1 segundos ). Es capaz cambiar el modo de envio entre el modo promediado y el modo instantáneo. 

La medición con el sensor BME688 se configuró para que el sensor BME688 obtenga los datos de temperatura y humedad, empleando comunicación I2C para la comunicación con el sensor.

## Compilación e implementación

Considerar que para implementar de manera correcta este código se debe utilizar ESP-IDF, además de configurar y actualizar las credenciales Wi-Fi y la IP del servidor en el archivo.

## Funciones clave

### Inicialización del sensor

```
esp_err_t sensor_init(void);
```

### Lectura de temperatura

```
uint32_t bme_read_data(void);
```

### Lectura de humedad

```
uint32_t bme_read_hum_data(void);
```

### Conexión TCP y envío de datos

```
void socket_tcp_CicloPrincipal(void);
```

### Control de Wi-Fi

```
void wifi_init_sta(char* ssid, char* password);
```

## Descripción del código en Python
Para el funcionamiento del código en c se debe ingresar al [código de la Raspberry Py](https://github.com/proyectostic3/TareasG4Tic3/blob/main/Tarea%202/InterfazPython/ServerInterface.py),
### Interfaz
