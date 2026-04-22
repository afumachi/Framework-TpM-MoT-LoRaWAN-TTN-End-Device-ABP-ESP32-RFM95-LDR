<img width="960" height="1280" alt="WhatsApp Image 2026-04-22 at 20 19 17" src="https://github.com/user-attachments/assets/43ff4cc8-dd0a-41c4-8a7b-0a98a85cda34" />


Electrical Engineering University of Campinas - FEE - UNICAMP

https://www.wisstek-iot.cc/

Extension Course - FEE-247 FEE-247 (DEVELOPMENT OF IOT SOLUTIONS WITH LORA AND LORAWAN)

This Repository is an IoT Solution using Three-Phase Methodology for a LoRaWAN End Device ABP connecting on TTN over a Single Channel LoRaWAN Gateway, by MoT protocol using a Device PKLoRa-WAN composed in base of ESP32 core & LoRa radio RFM95 The End Device LoRaWAN is composed in addtional of Sensors LDR (Light Intensity).

This repository is composed of 6 Levels of an IoT Solution developed through Three-Phase Methodology.

Level 1 - The LoRaWAN End Device ABP collects data from the sensor LDR and transmits this data via LoRa radio using the MoT protocol. This device is declared in a TTN application.

Level 2 - The LoRaWAN Single Channel Gateway, also declared in the TTN gateways, receives this data, and transmits them through Wi-Fi Internet connection to TTN Application Server.

Level 3 - The Edge Nivel_3 in Python, do a connection to TTN Application Server, and collects the data from the Application using an API Key login, via MQTT (TLS port 8883) and save the data from the LDR sensor in the Level 4 - Nivel4.csv file.

Level 4 - Nivel4.csv file. used to save all Uplinks data, as application (sensor LDR signal) and Network manager data (RSSI ans SNR).

Level 5 - Do the linear calculation of SNR aerage Data.

Level 6 - Dashboards - User application and IoT manager control. Also GPS position from End Device declared in it firmware.

Application Dashboard
<img width="1366" height="728" alt="Captura de tela 2026-04-22 204032" src="https://github.com/user-attachments/assets/b71ef3cb-3450-4d7a-a588-dc66d50a7858" />


Dashboard - LoRaWAN Network Management
<img width="1366" height="728" alt="Captura de tela 2026-04-22 204047" src="https://github.com/user-attachments/assets/ffe622b8-4250-4bf0-a768-2d60aa1b07b5" />


Dashboard - LoRaWAN End Device GPS Position
<img width="1366" height="728" alt="Nivel6-pklorawan" src="https://github.com/user-attachments/assets/81cd0fe6-3f38-4317-bf64-a4d7ea6a36ff" />
