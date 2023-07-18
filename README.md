# zeromq-experiment
We are trying to understand the feasibility of using 
[ZeroMQ](https://zeromq.org/) 
as the message library for software components of autonomous vehicles, like the
[F1/10](https://f1tenth.org/) car.  
This is a university project made for the subject "Real-Time Embedded Systems", for Università degli studi di Modena e Reggio Emilia. , year 2022-2023.

## Introduzione
TODO: Spiegare in maniera breve cos'è e come funziona ROS2  
TODO: Spiegare in maniera breve cos'è e come funziona ZeroMQ  
TODO: Spiegare l'obiettivo del paper

## Background
TODO: Creare una feature table in cui si confrontano i servizi offerti da ROS2 e quelli offerti da ZeroMQ  

||||
|---|---|---|
|Feature|ROS2|ZeroMQ|
|Node discovery|☑️|❌|

TODO: Capire e spiegare con degli schemi come funziona la comunicazione intraprocesso, interprocesso e internodo su ROS2
TODO: Spiegare con degli schemi come funziona la comunicazione intraprocesso, interprocesso e internodo su ZeroMQ
# Valutazione
## Ambiente di valutazione
TODO: Descrivere il setup dell'ambiente di sperimentazione:
- Le specifiche del PC
- Le specifiche della jetson
- Tutte le configurazioni del SO
- Ad esempio, io l'ubuntu 20 della jetson l'ho aggiornato
- La versione del compilatore
## Metodologia di valutazione
TODO: Spiegare in breve come è fatto il codice di benchmark.
- Riportare tutti gli sgami fatti, come isolcpu
- [Sgami](https://github.com/ZhenshengLee/ros2_jetson_benchmarks)  
- Riportare le flag del compilatore
## Risultati delle valutazioni
[issue](https://github.com/ElDavoo/zeromq-experiment/issues/1)
Qua grafici, dati, etc etc

# Conclusioni
ZeroMQ va bene per messaggi piccoli ed è più veloce, o forse no? 
