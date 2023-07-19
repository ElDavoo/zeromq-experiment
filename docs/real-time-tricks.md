# RT Tricks
This series of tricks do NOT have the goal of making the testing environment real-time,
but only to **reduce** the background noise in order to **improve** the test accuracy.  
# Disable the irqbalance daemon
The irqbalance daemon balances the IRQ between all CPUs
```sh
sudo systemctl disable --now irqbalance
```
# Disable extra stuff
## Disable the GUI
```sh
sudo jetson-config
```
- Select "desktop"
- Select "B1 - Console"
## Disable extra stuff
In my Jetson, teamviewer and docker were enabled, so:
```sh
sudo systemctl disable --now teamviewerd docker
```
# Kernel parameters
```
sudo nano /boot/extlinux/extlinux.conf

write: isolcpus=1-3 nohz_full=1-3 mitigations=off
in kernel parameters
reboot
```
# To execute at every boot
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
```
d
