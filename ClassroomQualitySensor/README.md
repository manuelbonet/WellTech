# Classroom quality sensor

## Parameters

The parameters that the classroom quality sensor measures are:
* Temperature (°C)
* Pressure (hPa)
* Relative humidity (%)
* Volume (dB)
* CO<sub>2</sub> concentration in air (ppm)

## Hardware

In terms of processing power, we decided to go for a TTGO LoRa32 with an OLED screen, which provides LoRaWAN, Bluetooth and WiFi (not used) communication as well as a handy screen from which to easily debug any possible problem.

To power the sensor, it is as simple as using a 5V 1A micro USB phone charger.

The sensors used for the project are:
* BME280 (with I<sup>2</sup>C output)
* Three electret microphones
  * One without any gain (i.e. the capsule itself)
  * One with adjustable gain set at minimum
  * One with adjustable gain set at maximum
* MH-Z14A
* Push button (to calibrate the CO<sub>2</sub>sensor if it were needed)

A functional diagram of the wiring can be found below.

![Wiring](https://raw.githubusercontent.com/manuelbonet/WellTech/master/ClassroomQualitySensor/images/Diagram%20-%20Classroom%20quality%20sensor.png)

## Software

The TTGO LoRa32 board can be programmed with Arduino IDE after installing the board through the board manager. The main code (classroomquality.ino) depends on the libraries that have to be added before trying to compile it. The libraries are property of their respective owners and they are under their respective licenses. I have uploaded them to the folder as a way to simplify finding and installing them.

The measured values are sent through the TTN network as comma-separated values and they have to be parsed before they arrive to the ThingSpeak integration. To decode and convert this data, the following snippets are needed under "Payload format":

```javascript
function Decoder(bytes, port) {
  while (bytes[bytes.length - 1] === 0){
    bytes.pop();
  }
  
  text = "";
  
   for (i = 0; i < bytes.length; i++){
     text += String.fromCharCode(bytes[i]);
   }
   
   return {"text": text};
```

```javascript
function Converter(decoded, port) {
  
   dividedText = decoded["text"].split("/");
   
   temperature = parseFloat(dividedText[0]);
   pressure = parseFloat(dividedText[1]);
   humidity = parseFloat(dividedText[2]);
   volume = parseFloat(dividedText[3]);
   CO2 = parseFloat(dividedText[4]);
   
   convertedData = {};
   
   if(!isNaN(temperature)){
     convertedData["field1"] = temperature;
   }
   
   if(!isNaN(pressure)){
     convertedData["field2"] = pressure;
   } 
   
   if(!isNaN(humidity)){
     convertedData["field3"] = humidity;
   }
   
   if(!isNaN(volume)){
     convertedData["field4"] = volume;
   }
   
   if(!isNaN(CO2)){
     convertedData["field5"] = CO2;
   }
  
  return convertedData;
}
```

These two snippets are prepared to have the ThingSpeak channels in the same order as written in the [Parameters](#parameters) section.

## Sound calibration

Due to the fact that electret microphones do not output the volume of the sound they capture in decibels, a relationship between the peak-to-peak voltage and the volume needs to be found. [Here](https://docs.google.com/spreadsheets/d/1mHTgkDXV53TCKcUPq5pmppE6GQpHShiHMCgk4wV-P8s/edit?usp=sharing) is a spreadsheet that calculates this equivalence given some empirical measurements.

## Box

The sensors have to fit in a box, which should have enough holes so that both air and sound can easily get inside. We have provided a box which directly fits a 15x9 cm perfboard and holes big enough to achieve this.

## Cost

This is a cheaper project, its cost is of around €45. The two main costs of the classroom quality sensor are the TTGO LoRa32 and the MH-Z14A, each costing around €17.
