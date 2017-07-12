# Soldering Station 

Diy Estación de soldado con Atmega328 (Arduino UNO), compatible con HAKKO 936/937/928/926

## Información

Estacion de soldado hecho sobre un Atmega328, utilizando un Lapiz de una estación Hakko.

A diferencia de la estación original, esta alimenta a la Punta/Lapíz con corriente alterna (CA, 24Vac).

ATENCIÓN: Tener en cuenta que este Lapiz utiliza un PTC para el sensado de la temperatura y NO una termocupla.

<p align="center">
  <br><br>
  <b>Lapiz utilizado</b><br>
  <img src="https://github.com/kr4fty/DiySolderingStation/blob/master/hardware/lapiz.png">
  <br>
</p>


## Caracteristicas

  * ESD, Antiestático
  * Tensión: 24 Vac 
  * Rango de temperaturas: 120-480 ºC.
  * Potencia: 50 W

## Partes

  * LCD del tipo I²C: se utilizó un lcd de un Motorola C115
  * Encoder rotatorio
  * Punta/lapiz de Estación de soldado, compatible con HAKKO 907
  * Botón para chasis
  * Controlador (incluido en el repositorio)

## Software

Depende de las librerias:
  * [ArduinoPID](https://github.com/br3ttb/Arduino-PID-Library): Librería para el control de la temperatura por medio de un PID
  * [ST7558](https://github.com/kr4fty/ST7558-Motorola-C115-LCD-Library): Driver para manejar el LCD
  * [TimerOne](https://github.com/PaulStoffregen/TimerOne): para el Control de Fase de la tensión de alimentación
  * [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt): Para habilitar tanto las interrupciones externas como las del tipo Pin-Change
  * [EncoderPCI](https://github.com/kr4fty/EncoderPCI): Se encarga de manejar el Encoder

## Hardware

<p align="center">
  <br><br>
  <b>Circuito eléctrico</b><br>
  <img src="https://github.com/kr4fty/DiySolderingStation/blob/master/hardware/circuit.png">
  <br>
</p>

