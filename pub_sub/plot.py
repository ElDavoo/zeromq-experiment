# Plot usando matplotlib anzichè l'applicazione Plotjuggler;
# Non testata!

import matplotlib.pyplot as plt
import zmq
import json

# Inizializza il contesto ZeroMQ
context = zmq.Context()

# Crea un socket SUB per ricevere i dati
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5565")
socket.setsockopt(zmq.SUBSCRIBE, b"")

# Inizializza le liste per i dati
x_data = []
y_data = []

# Funzione per plottare i dati
def plot_data():
    plt.plot(x_data, y_data)
    plt.xlabel('Asse X')
    plt.ylabel('Asse Y')
    plt.title('Grafico dei dati di ZeroMQ')
    plt.show()

# Ricevi e plotta i dati
while True:
    data = socket.recv()
    # Elabora i dati ricevuti
    json_data = json.loads(data)  # Analizza la stringa JSON in un dizionario Python
    x_value = json_data['count']  # Estrai il valore di 'x' dal dizionario
    y_value = json_data['rtt']  # Estrai il valore di 'y' dal dizionario
    # Aggiungi i dati alle liste
    x_data.append(x_value)
    y_data.append(y_value)
    # Plotta i dati aggiornati
    plot_data()
