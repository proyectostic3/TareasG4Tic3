Este es un repositorio que contiene el desarrollo de la Tarea 1 y 2 por parte del grupo 4, de la asignatura **Taller de Integración TIC** con código 549353-1. Sus **integrantes** son:
- Tomás Troncoso
- Javiera Mora 

<h1>Descripción proyecto Final (Tarea 2)</h1>

## Descripción general del código en c
Para vizualizar el código en C se debe ingresar al [código del ESP32](https://github.com/proyectostic3/TareasG4Tic3/tree/main/Tarea%202/TIC3ProyectoFinal/main/ProjectoFinal_TIC3.c), este último realiza la conexión TCP a través de una red Wi-Fi con un servidor en Raspberry Py, mediante esta conexión se envían los datos de temperatura y humedad empleando el sensor BME688  empleando comunicación I2C para la comunicación con el modulo ESP32. Para la obtención de la data se empleó una serie de funciones para extraer y leer correctamente los valores de tanto de la temperatura en celcuis como la humedad relativa (medida en porcentaje). 

Notar que además se emplea un LED, el cual cumple la función de indicador, puesto que parpadea durante el envío de datos (cuando ocurre la conexción TCP), además de que cambia si el usuario a través de la interfaz indica una tasa de muestreo de datos. Esta comunicación es capaz de iniciarse o detenerse respecto al envio de datos. 

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
Para visualizar el código en Python se debe ingresar al [código de la Raspberry Py](https://github.com/proyectostic3/TareasG4Tic3/blob/main/Tarea%202/InterfazPython/ServerInterface.py), es mediante este código que se realiza el promedio entre el maximo y minimo valor obtenido a una tasa promedio de 500 Hz tanto de la temperatura como de la humedad, estos valores se pueden visualizar graficados a la parte izquierda de la interfaz. 

Por otra parte en la parte derecha se encuentran los valores medidos, como tambien una serie de botones que puedes 
