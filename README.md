# RaspberryPi-Setup
Be Companion Computer for Pixhawk

Hardware:Raspberry Pi 3

OS:[RASPBIAN JESSIE LITE (2016-05-10)](https://www.raspberrypi.org/downloads/raspbian/)

Note:[INSTALLING OPERATING SYSTEM IMAGES](https://www.raspberrypi.org/documentation/installation/installing-images/README.md)

## Common

Using

```bash
sudo raspi-config
```

to change locale/time etc.

```bash
sudo nano /etc/apt/sources.list
```

Replace the original address with USTC mirror

```bash
deb http://mirrors.ustc.edu.cn/raspbian/raspbian/ jessie main contrib non-free rpi
deb-src http://mirrors.ustc.edu.cn/raspbian/raspbian/ jessie main contrib non-free rpi
```

```bash
sudo apt-get update
```

```bash
sudo apt-get upgrade
```

```bash
sudo apt-get install rpi-update htop zsh git git-flow vim
```

```bash
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
```

Then enter password to change to zsh or you need

```bash
chsh -s /bin/zsh
``` 

and reboot later

```bash
curl http://j.mp/spf13-vim3 -L -o - | sh
```

```bash
sudo rpi-update
```

```
sudo reboot
```

## Config a Wi-Fi dongle

```bash
sudo nano /etc/network/interfaces
```

modify content like below:

```bash
auto lo
iface lo inet loopback
iface eth0 inet dhcp
auto wlan0
allow-hotplug wlan0
iface wlan0 inet dhcp
wpa-ssid “YOUR SSID”
wpa-psk “YOUR PASSWORD”
```

```bash
sudo /etc/init.d/networking restart
```

Then

```bash
ifconfig
```

to check wlan0's IP address

#Apt-get install
```bash
sudo apt-get install python-dev python-pip git vim
sudo apt-get -y install gpsd gpsd-clients python-gps
```

#pip install
```bash
sudo pip install paho-mqtt threadpool pynmea2
sudo pip install --pre azure
```
[Azure python SDK](https://github.com/Azure/azure-sdk-for-python)


#Test GPS module
```bash
sudo dpkg-reconfigure gpsd
sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock
sudo killall gpsd
sudo cgps -s

```

