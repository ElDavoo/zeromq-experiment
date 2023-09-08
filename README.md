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
![ROS2 Bozza di schema](https://github.com/ElDavoo/zeromq-experiment/assets/4050967/5eca201f-98d8-4ffa-913b-0ac451c3761a)
# Valutazione
## Ambiente di valutazione
TODO: Descrivere il setup dell'ambiente di sperimentazione:
- Le specifiche del PC
- Le specifiche della jetson
```sh
root@nano:~# lsb_release -a
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal

sudo jetson-release

Software part of jetson-stats 4.2.2 - (c) 2023, Raffaello Bonghi
Model: NVIDIA Jetson Nano Developer Kit - Jetpack 4.6 [L4T 32.6.1]
NV Power Mode[0]: MAXN
Serial Number: [XXX Show with: jetson_release -s XXX]
Hardware:
 - P-Number: p3448-0000
 - Module: NVIDIA Jetson Nano (4 GB ram)
Platform:
 - Distribution: Ubuntu 20.04 focal
 - Release: 4.9.253-tegra
jtop:
 - Version: 4.2.2
 - Service: Inactive
Libraries:
 - CUDA: 10.2.300
 - cuDNN: 8.2.1.32
 - TensorRT: 8.0.1.6
 - VPI: 1.1.15
 - Vulkan: 1.2.141
 - OpenCV: 4.6.0 - with CUDA: YES
```
- Tutte le configurazioni del SO
- Ad esempio, io l'ubuntu 20 della jetson l'ho aggiornato
- La versione del compilatore
## Metodologia di valutazione
TODO: Spiegare in breve come è fatto il codice di benchmark.
- Seguire [real-time-tricks](docs/real-time-tricks.md)
- Riportare le flag del compilatore
## Risultati delle valutazioni
[issue](https://github.com/ElDavoo/zeromq-experiment/issues/1)
Qua grafici, dati, etc etc

# Conclusioni
ZeroMQ va bene per messaggi piccoli ed è più veloce, o forse no? 
