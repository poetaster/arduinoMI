# Cloudy Plaits RP2350

This is example code of **Mutable Instruments Plaits 1.2** and **Clouds Parasites firmware reverb** running on **Raspberry Pi Pico RP2350** with **I2S DAC**.

Program cycles through clean Plaits engines -> Clouds reverb -> Oliverb fx

An additional **DX7 SysEx bank** has been added for the OP6 engine, making a total of **25 engines** available.

## Setup Instructions
In your Arduino IDE library folder (~/Arduino/libraries) if you already have the libraries, just use the checkout commands.

git clone git@github.com:poetaster/CLOUDS.git
cd CLOUDS
git checkout cloudsparasites 

git clone git@github.com:poetaster/PLAITS.git
cd PLAITS
git checkout cloudsparasites 

git clone git@github.com:poetaster/STMLIB.git 
cd STMLIB
git checkout cloudsparasites 


- CPU: **276 MHz Overclock**  
- Optimize: **Even More -O3**  
- Select **TinyUSB stack** in ArduinoIDE tools for easier adjustments and testing  

## License & Authorship

**Vadims Maksimovs** — [github.com/ledlaux](https://github.com/ledlaux) (2025)  
License: MIT

This code is part of the porting project of Mutable Instruments eurorack modules to Arduino by **Mark Washeim**, licensed under the MIT License

Main repository: [arduinoMI](https://github.com/poetaster/arduinoMI)

### Original Mutable Instruments Code

 Author: **Emilie O. Gillet** (2020)  
 Libraries: stmlib, Plaits, Clouds  
 License: MIT

### Clouds Parasites Firmware
 Author: **Matthias Puech**  
 Website: [mqtthiqs.github.io](https://mqtthiqs.github.io/)  
 License: MIT
