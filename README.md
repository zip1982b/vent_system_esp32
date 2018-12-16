Project name: vent_system_esp32
This project is based on Espressif IoT Development Framework (esp-idf).

autor: Zhan Beshchanov<zip1982b@gmail.com>

Ventilation control project with heat exchanger.

Hardware:
* esp32
* ds2482-100 slave (task -)
* 2 BT136
* 2 MOC3021
* 1 PC817



Connections:
* GPIO34 zero sensor (pc817)
* GPIO32 fan control FAN_IN
* GPIO33 fan control FAN_OUT 


  i2c slave
* DS2482 SCL - GPIO19
* DS2482 SDA - GPIO18



