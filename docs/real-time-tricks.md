# RT Tricks
Notare che questi parametri servono a ridurre il rumore di fondo e ad 
aumentare quindi la precisione dei benchmark, ma non a rendere il sistema real-time. 
# Disattivare irqbalance
irqbalance è un demone che bilancia i carichi di lavoro tra i core della CPU.
```sh
sudo systemctl disable --now irqbalance
```
# Disattivare cose non necessarie
## Disattivazione interfaccia grafica
```sh
sudo jetson-config
```
- Selezione "desktop"
- Selezione "B1 - Console"
## Altri programmi
Nella nostra Jetson, questi programmi erano attivi e sono stati disattivati:
```sh
sudo systemctl disable --now teamviewerd docker
```
# Parametri del kernel
```
sudo nano /boot/extlinux/extlinux.conf

mettere: isolcpus=1-3 nohz_full=1-3 mitigations=off
nei parametri del kernel
reboot
```
# Eseguire ad ogni avvio
```sh
sudo -i 
sysctl vm.stat_interval=120
echo never > /sys/kernel/mm/transparent_hugepage/enabled
swapoff -a
sysctl -w net.ipv4.udp_mem="10240087380016777216"
sysctl -w net.core.netdev_max_backlog="30000"
sysctl -w net.core.rmem_max="67108864"
sysctl -w net.core.wmem_max="67108864"
sysctl -w net.core.rmem_default="67108864"
sysctl -w net.core.wmem_default="67108864"
sysctl -w net.inet.udp.recvspace=209715
sysctl -w net.inet.udp.maxdgram=65500
```