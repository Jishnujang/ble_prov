- In this project I used three Examples of ESP idf

  1.GATT server
  2.Wifi station
  3.Mqtt client

*we can use esp32 flash download tool to download the application to ESP32.
*Additional required data are
address of bootloader 0x1000
address of partion table 0x8000
address of factory/application code 0x10000
Broker : mqtt://topic:aio_tjHu05CFKDQ1o4HSqhboBrLlG1RY@io.adafruit.com
uuid: 0x00ff,0x00DD,0x00EE
Also we can use IDF commands to build, select target, select port,flash and monitor.
