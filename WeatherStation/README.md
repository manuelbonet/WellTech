# Weather station

## Parameters

The parameters that the weather station provides are:
* Temperature (°C)
* Apparent temperature (°C)
* Pressure (hPa)
* Relative humidity (%)
* Dew point (°C)
* Wind speed (km/h)
* Wind direction (in increments of 22.5°)
* CO<sub>2</sub> concentration in air (ppm)

## Hardware

In terms of processing power, we decided to go for a TTGO LoRa32 with an OLED screen, which provides LoRaWAN, Bluetooth and WiFi (not used) communication as well as a handy screen from which to easily debug any possible problem.

Regarding powering the weather station, we went for a 12V solar panel. This is because we wanted to be able to place it in the most optimal location to take good readings without having to be close to a power outlet. Dtue to this, we also had to create a sturdy structure in order to place the solar panel on top. The reason why we chose a 12V system specifically is because the anemometer and the wind vane run at that voltage, but because the CO<sub>2</sub> sensor and the TTGo LoRa32 need to be powered with 5V we also had to include a 12V-5V buck converter. All other components run at 3.3V, but the TTGO LoRa32 board generates this voltage.

The sensors used for the project are:
* DS18B20
* BME280 (with I<sup>2</sup>C output)
* Anemometer and wind vane (we used [these](https://es.aliexpress.com/item/33019729997.html), specifically the ones that output a 0-5V signal)
* Trimmer potentiometer (to calibrate the wind direction)
* MH-Z14A
* Push button (to calibrate the CO<sub>2</sub> sensor if it were needed)

A functional diagram of the wiring can be found below.

![Wiring](https://raw.githubusercontent.com/manuelbonet/WellTech/master/WeatherStation/images/Weather%20station%20diagram.jpg)

## Software

The TTGO LoRa32 board can be programmed with Arduino IDE after installing the board through the board manager. The main code (weatherstation.ino) depends on the libraries that have to be added before trying to compile it. The libraries are property of their respective owners and they are under their respective licenses. I have uploaded them to the folder as a way to simplify finding and installing them.

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
}
```

```javascript
function Converter(decoded, port) {
  
   dividedText = decoded["text"].split("/");
   
   temperature = parseFloat(dividedText[0]);
   apparentTemperature = parseFloat(dividedText[1]);
   pressure = parseFloat(dividedText[2]);
   humidity = parseFloat(dividedText[3]);
   dewPoint = parseFloat(dividedText[4]);
   windSpeed = parseFloat(dividedText[5]);
   windDirection = parseFloat(dividedText[6]);
   CO2 = parseFloat(dividedText[7]);
   
   convertedData = {};
   
   if(!isNaN(temperature)){
     convertedData["field1"] = temperature;
   }
   
   if(!isNaN(apparentTemperature)){
     convertedData["field2"] = apparentTemperature;
   }
   
   if(!isNaN(pressure)){
     convertedData["field3"] = pressure;
   }
   
   
   if(!isNaN(humidity)){
     convertedData["field4"] = humidity;
   }
   
   
   if(!isNaN(dewPoint)){
     convertedData["field5"] = dewPoint;
   }
   
   if(!isNaN(windSpeed)){
     convertedData["field6"] = windSpeed;
   }
   
   if(!isNaN(windDirection)){
     convertedData["field7"] = windDirection;
   }
   
   
   if(!isNaN(CO2)){
     convertedData["field8"] = CO2;
   }
  
  return convertedData;
}
```

These two snippets are prepared to have the ThingSpeak channels in the same order as written in the [Parameters](#parameters) section.

## Best practices for measurements

There are several things that should be considered when deciding the structure and the location for the weather station collected in the World Meteorological Organizations' [Guide to Meteorological Instruments and Methods of Observation](https://library.wmo.int/index.php?lvl=notice_display&id=12407). However, some of these requirements turn out to be very difficult to satisfy with a setup like ours, especially those related to wind. Nevertheless, the data collected with such a weather station, although not perfect, is more than good enough to be able to learn from.

Given that these criteria are almost impossible to follow in the context of a school, it is very important to place the anemometer and vane as high and as far away as possible from any obstacle even if they are not met. Therefore, we suggest placing the weather station as far away from any tall buildings as possible and having the anemometer and wind vane at a height of at least 2 m.

## Solar power

If the weather station is solar powered, other factors should also be considered, the main one being that the weather station should be placed in such a place where ideally no shadow can be cast over the solar panel.

In terms of which solar panel and battery to choose, it is important to calculate their sizes. There are websites online and mobile apps that make this process easier. The power consumption of the weather station (with a 30% safety margin) is of 1.8W or 43.2Wh/day. Regarding the position of the solar panel, it should be pointing towards the South in the Northern Hemisphere and the North in the Southern Hemisphere, and its tilt should be appropriate to the season of year and the geographical latitude.

## Cost

This is not a cheap project, it cost us around €300 in components to build it. However, there are things that can be removed to lower the price to a much more affordable price point.

Powering the weather station through solar power is one of the main extra costs. Around €100 of that price comes directly from the solar panel, the charge controller and the battery we used. Removing the solar panel would also mean that the cost of the structure (€100) could decrease as it would not have to hold these components.

The other main source of price comes from the anemometer and the wind vane, which cost around €65. If the wind vane were not to be used, the trimmer potentiometer would no longer be necessary. However, in order to do this properly, the main code and the TTN snippets would have to be changed.
