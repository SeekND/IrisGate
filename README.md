# IrisGate
Iris gate controlled by RSSI




## You can see a video of it working here: ##
https://www.youtube.com/watch?v=XlLPElB3BOo





## Materials you need

> 1x Arduino Nano (or an uno if you have one should also work)
https://www.banggood.com/ATmega328P-Arduino-Compatible-Nano-V3-Improved-Version-No-Cable-p-959231.html

> 1x FPV Receiver module with SPI mod (these below already have the mod but double check - google vtx spi mod or check the pictures in this repo)
https://www.banggood.com/FPV-5_8G-Wireless-Audio-Video-Receiving-Module-RX5808-p-84775.html

> 1x servo
Recommend above 9g unless your iris gate is super lubbed ;)

> 1x Iris Gate
Make sure the mechanical part of the iris is working effortlesslly before installing the servo and arduino
Example of an iris gate: https://makezine.com/projects/mechanical-iris/



## How to assemble it

1. Now solder the Channel 1 pin in your VTX receiver to your Arduino D10
2. Solder the Channel 2 pin in your VTX receiver to your Arduino D11
3. Solder the Channel 3 pin in your VTX receiver to your Arduino D12
4. Solder the Ground pin in your VTX receiver to your Arduino Ground (there are several, pick any)
5. Solder the 5v pin in your VTX receiver to your Arduino 5V
6. Solder the RSSI pin in your VTX receiver to your Arduino A3
7. And finally, solder the servo to Arduino 5v, ground and solder the signal cable to Arduino D5



## How to upload the code

1. Start by downloading and installing Arduino IDE (https://www.arduino.cc/en/Main/Software)
2. Next, download THIS repository and unzip it somewhere in your desktop.
3. Go to the new folder you downloaded and open the file called IRIS.INO
4. You will now be able to see a bunch of code.
5. Goto Tools/Board and pick Arduino Nano (if you bought an arduino nano, otherwise pick the one you have). The processor is ATMega328P.
6. Now to go port and pick the port your Arduino Nano is connected to. If you don't know which one it is, disconnect the nano and check what ports are available. Reconnect the nano, the new port will be the nano. If you don't get any new ports I'm afraid you will need to google how to troubleshoot it :)
7. VERY IMPORTANT. You will need to play with the RSSITHRESHOLD value to find what works best for you. I find that most receivers work differently and that value can be between 190 to 260. Test it out with with EACH nano you build
8. Finally, when you are ready, goto sketch menu and select upload. If you done everything right you will see an increasing bar on the bottom right handside of the program and then a message saying Done uploading on the lower left handside.




# Congratulations!
You are now the proud owner of a IRIS GATE!
