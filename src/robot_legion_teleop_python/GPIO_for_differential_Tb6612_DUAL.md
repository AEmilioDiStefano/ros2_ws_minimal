# GPIO connections from Raspberry Pi 4 to DUAL Tb6612fng motor controllers to motors for a TANK TRACK DIFFERENTIAL DRIVE Chassis  

### **This version wires each SIDE as a paired set**:  
###   - Left Front + Left Rear receive the SAME PWM + direction signals  
###   - Right Front + Right Rear receive the SAME PWM + direction signals  
###   This prevents motors from fighting each other in a tank-style track system.  

#  
#  

### TB6612 #1 - LEFT SIDE MOTORS (paired together)  

**Left Front Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**Left Rear Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections (tied so BOTH channels get identical commands)**  

PWMA **and** PWMB tied together → GPIO 12 (PWM)  

AIN1 **and** BIN1 tied together → GPIO 5  

AIN2 **and** BIN2 tied together → GPIO 6  

**Standby (board enable)**  

STBY to VCC (same motor driver)

#  
##  

### TB6612 #2 - RIGHT SIDE MOTORS (paired together)  

**Right Front Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02   

**Right Rear Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections (tied so BOTH channels get identical commands)**  

PWMA **and** PWMB tied together → GPIO 18 (PWM)  

AIN1 **and** BIN1 tied together → GPIO 23  

AIN2 **and** BIN2 tied together → GPIO 24  

**Standby (board enable)**  

STBY to VCC (same motor driver)  

#  
#  

### Logic power  

Pi 3.3V to VCC on both TB6612 boards  

Pi GND to GND on both boards  

**(3.3V logic is ideal with Pi GPIO)**

#  
#  

### Motor power  

**If you’re using your 2×18650 (2S) pack**:  

Battery + to VM on both boards  

Battery – to GND on both boards  

#  
#    

### Common ground (critical)  

**All grounds must connect together**:  

Pi GND to TB6612 GND to Battery – (negative)  

#  
#    

### STBY (enable)   
  
On TB6612 #1: STBY to VCC (same board)  
On TB6612 #2: STBY to VCC (same board)  


