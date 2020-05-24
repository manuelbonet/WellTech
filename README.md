# WellTech

WellTech is a school project developed by Manuel Bonet, Martín Subirá and Pablo Mellado in order to use technology as a way to improve our daily lives as students and change the ways we learn. To achieve this goal, we decided to create two different devices: a weather station and a classroom quality sensor.

The classroom quality sensors measure parameters that we consider key to be able to study in a comfortable environment such as temperature, humidity, sound volume and the CO<sub>2</sub> concentration in air. This last value might seem weird, but there are several studies which have found a relationship between the level of carbon dioxide in the air and the cognitive ability of a person. Given that this is precisely an ability that students need to put in practice, it is a very important parameter to control.

We have also created a weather station that monitors data such as temperature, pressure, humidiity, wind speed and direction and CO<sub>2</sub> concentration. This is because we consider that a very effective way to have students learn information is through practical activities which can be connected to as many subjects as possible. We believe that such a project could be used in a variety of subjects such as Technology (creation of the structure and the circuit), Computer Science (coding), Physics and Chemistry (understanding of meteorological phenomena) and Mathematics (statistics, processing of the raw data). Another advantage to such a project is that pupils have an active role in learning which makes it more difficult to distract themselves, and being able to collect their own data provides an incentive not present in traditional theoretical exercises.

The data collected by these two devices is sent through the [TheThingsNetwork](https://www.thethingsnetwork.org/) LoRaWAN network and the information is stored using [ThingSpeak](https://thingspeak.com/). As a way to debug any possible problem there might be, they also send the data they collect through the serial port and through Bluetooth with the pairing code 2468.
