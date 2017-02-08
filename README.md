# iridium_arduino
Who am I?
Universitat Politècnica de Catalunya (UPC - BarcelonaTECH) - ESEIAAT
EUROAVA TERRASSA// UPSC SPACE PRGOGRAM
CODE DEVELOPED FOR "EVERY CHILD WANTS TO BE AN ASTRONATUT" UPC SPACE PROGRAM
Iridium + GPS UBLOX8M + BMP180 + 2 Relays(optional) + Dallas temperature Sensor
This code is meant to be used in HABs type missions.
It consists a IRIDIUM modem, to send messages over the world, a GPS Ublox M8 with flight mode,
a BMP180 barometric-temperature 
What we are sending
We send time, longitude (N/S), latitutde (E/W), 
number of GPS satel used, altitude GPS, state (0 or 1, depending on if we have GPS signal)
Temperature BMP180, altitude BMP180 (just in case GPS altitude is not so accurated) and temp from LM36 (outside). Also, redundant systems are always good.
Instructions
First, configure IRIDIUM. Make sure you have enough credits to use this modem, going to https://rockblock.rock7.com/Operations 
and login. Please make sure that if we are sending very large messages, we could spend 2 credits instead of 1. 
Calculte total time of the launch, add a safety factor of 1.5, and buy messages enough to fullfill the mission.
IT IS NOT RECCOMENDED BUY CREDITS WHEN IRIDIUM IS WORKING. IRIDIUM COULD STOP WORKING. BUY CREDITS BEFORE THE MISSION!  

Questions, please refer to marc.cortes.fargas@upcprogrm.space (prefered) or marc.c.fargas@gmail.com
Made by Marc Cortés Fargas.
This code is under GNU GENERAL PUBLIC LICENSE. This program comes with absolutely no warranty, provided as it is. 
Sparkfun's BMP180 library is used, as well as PString and IRIDIUMSBD developed for others. All credit for this libraries to their owners!


REV-25.01.201