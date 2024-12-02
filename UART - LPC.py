import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Configuración UART
uart_port = "COM7"  # Cambia esto al puerto UART correspondiente
baud_rate = 9600
ser = serial.Serial(uart_port, baud_rate, timeout=1)

# Diccionario para asignar alturas a las notas
notas = {"DO": 1, "RE": 2, "MI": 3, "FA": 4, "SOL": 5, "LA": 6, "SI": 7, "DO2": 8, "RE2": 9, "MI2": 10, "FA2": 11, "SOL2": 12, "LA2": 13, "SI2": 14, "DO3": 15, "RE3": 16}
data = []  # Lista para almacenar las notas recibidas

# Configuración del gráfico
fig, ax = plt.subplots()
bars = ax.bar(list(notas.keys()), [0] * len(notas), color='blue')

def update(frame):
    global data

    # Leer dato desde UART
    if ser.in_waiting > 0:
        time.sleep(0.05)
        line = ser.readline().decode('utf-8').strip()  # Leer nota enviada por UART
        print("Nota recibida:", line)  # Mostrar nota en la consola

        # Agregar la nota a la lista si es válida
        if line in notas:
            data.append(line)

    # Actualizar gráfico
    counts = {key: data.count(key) for key in notas.keys()}  # Contar ocurrencias de cada nota
    for i, bar in enumerate(bars):
        bar.set_height(counts[list(notas.keys())[i]])
    
    return bars

# Configuración inicial del gráfico
ax.set_ylim(0, 10)  # Altura máxima del gráfico
ax.set_title("Visualización de Notas Musicales (UART)")
ax.set_ylabel("Cantidad de Notas")
ax.set_xlabel("Notas")

# Animar gráfico
ani = FuncAnimation(fig, update, interval=50, save_count=100)  # Establecer un límite para el cache
plt.show()
