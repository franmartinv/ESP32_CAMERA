# ESP32-Camera

This project was done for implement in ESP32-CAM modules following the realization of my Final Degree Project in Electronic Industry and Automatic Engineering at University of Almería (Spain).

You can see the project of each sensor in my Github profile!

Name: Francisco Martín Villegas

Year: 2021-2022

Email: f.martinvillegas00@gmail.com


## Menuconfig configuration

This project is based on MQTT TCP example for ESP-IDF v4.4, because I need that the MQTT and Wifi protocols were good configured.

In this case you must enter in menuconfig to edit the Wifi SSID and password and the MQTT broker url.

Just open your ESP-IDF terminal, go with (`cd C:\.....`) to your projects' directory and type (`idf.py menuconfig`). You can change these here.

It's very important to enable SPIRAM for correct operation of the camera. Choose use 80 MHz of SPIRAM too!!

Only enjoy it!! And ask me on my email if you have a question!!
