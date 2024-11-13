#sudo apt update
#sudo apt install hostapd dnsmasq -y
#Configura hostapd para crear el punto de acceso Wi-Fi.
#Configura dnsmasq

import os
import subprocess

def setup_wifi(ssid, password):
    # 1. Configurar el archivo hostapd.conf
    hostapd_conf = f"""
interface=wlan0
driver=nl80211
ssid={ssid}
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase={password}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
    """

    with open('/etc/hostapd/hostapd.conf', 'w') as f:
        f.write(hostapd_conf)
    
    # 2. Configurar el archivo de dnsmasq.conf para DHCP
    dnsmasq_conf = """
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
    """
    
    with open('/etc/dnsmasq.conf', 'w') as f:
        f.write(dnsmasq_conf)

    # 3. Configurar el archivo de red para IP estática en wlan0
    network_config = """
interface wlan0
static ip_address=192.168.4.1/24
    """
    
    with open('/etc/dhcpcd.conf', 'a') as f:
        f.write(network_config)

    # Reiniciar el servicio dhcpcd para aplicar cambios
    subprocess.run(['sudo', 'systemctl', 'restart', 'dhcpcd'])

    # 4. Habilitar y iniciar los servicios hostapd y dnsmasq
    subprocess.run(['sudo', 'systemctl', 'unmask', 'hostapd'])
    subprocess.run(['sudo', 'systemctl', 'enable', 'hostapd'])
    subprocess.run(['sudo', 'systemctl', 'start', 'hostapd'])
    subprocess.run(['sudo', 'systemctl', 'restart', 'dnsmasq'])
    
    print("Punto de acceso configurado con éxito.")

# Ejemplo de uso
setup_wifi('MiRedWifi', 'contraseña1234')
