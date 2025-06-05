# Trabalho A2 - IOT

Código fonte do firmware utilizado na esp32-c6 devkit, para utilizar esse código lembre-se de atualizar o SSID e PASS da rede wifi para que se conecte, também é necessário um broker MQTT para comunicação com o software utilizado.

## Requisitos

- ESP32-C6 DevKit
- Broker MQTT (ex: Mosquitto)
- Rede Wi-Fi disponível

## Configuração

1. Clone este repositório:
    ```bash
    git clone https://github.com/seu-usuario/firmwareIOT_A2.git
    ```
2. Atualize o arquivo de configuração com o SSID e senha da sua rede Wi-Fi.
3. Configure o endereço do broker MQTT no código.

## Instalação

1. Instale o ESP-IDF (utilizei no VSCode).
2. Conecte a placa ESP32-C6 ao computador.
3. Compile e faça o upload do firmware para a placa.

## Uso

- O dispositivo irá conectar à rede Wi-Fi e ao broker MQTT configurado.
- Os tópicos MQTT utilizados podem ser ajustados conforme a necessidade do seu software.
