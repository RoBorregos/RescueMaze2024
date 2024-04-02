import socket
import csv

UDP_PORT = 1237

with open("received_messages.csv", "a", newline="") as csvfile:
    fieldnames = ["Timestamp", "Message"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Si el archivo está vacío, escribir la fila de encabezado
    if csvfile.tell() == 0:
        writer.writeheader()
        
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT))

    while True:
        data, addr = sock.recvfrom(1024)  # Tamaño del búfer: 1024 bytes
        message = data.decode('utf-8')

        # Obtener la hora actual (puedes usar una librería de fecha y hora más avanzada)
        timestamp = "2024-03-28 14:32:41"  # Cambia esto a la hora actual

        # Escribir la fila en el archivo CSV
        writer.writerow({"Timestamp": timestamp, "Message": message})

        print((f"Received message: {message}"))