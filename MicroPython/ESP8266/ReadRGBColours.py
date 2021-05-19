# Board - Lolin NodeMCU V3

from machine import Pin
from machine import UART
from machine import disable_irq
from machine import enable_irq
from machine import time_pulse_us
from machine import idle
import time

freqOutPin = Pin(4, Pin.IN, Pin.PULL_UP) # D2
isCaptureDone = False
doCalibration = False
edgeCount = 0
pulseWidth = 0
captures = [0, 0]
RED_MIN = 15
RED_MAX = 250
GREEN_MIN = 25
GREEN_MAX = 380
BLUE_MIN = 20
BLUE_MAX = 310

def mapToRange(x, in_min, in_max, out_min, out_max):

    in_max += 1
    out_max += 1

    ret = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    if((ret < out_min) and (out_min > out_max)):
        return int(ret)
    
    elif((ret < out_min) and (out_min < out_max)):
        return int(out_min)

    elif((ret > out_max) and (out_min > out_max)):
        return int(ret)
        
    elif((ret > out_max) and (out_min < out_max)):
        return int(out_max)
        
    else:
        return int(ret)
    

def inputCapture(triggeredPin):
    
    global captures
    global isCaptureDone
    global pulseWidth
    global edgeCount
    
    #print("input capture called\r\n")
    
    if not isCaptureDone:
        pulseWidth = time_pulse_us(freqOutPin, 1, 1000000)
        isCaptureDone = True
        #print(pulseWidth)
    
        

def uartSetup(baud=115200):
    return UART(1, baudrate=baud) # TX Only on UART1 GPIO2 D4 (use FTDI)
    
def gpioSetup():
    s0 = Pin(15, Pin.OUT) # D8
    s1 = Pin(13, Pin.OUT) # D7
    s2 = Pin(12, Pin.OUT) # D6
    s3 = Pin(14, Pin.OUT) # D5
    
    global freqOutPin
    
    # Caputre the falling edge and use time_pulse_us in inputCapture callback
    # to measure the length in us of high logic level pulse width. This is done
    # because the time_pulse_us routine starts counting if the pin was already high
    # so make sure that the pin was low before calling time_pulse_us
    freqOutPin.irq(trigger=Pin.IRQ_FALLING, handler=inputCapture)
    
    return (s0, s1, s2, s3)


def setFreqScaling(s0, s1, scaleFactor):
    
    scaleFactor100PCT = 100 # Max output freq of device is typically 600kHz
    scaleFactor20PCT = 20   # Max output freq of device is typically 120kHz
    scaleFactor2PCT = 2     # Max output freq of device is typically 12kHz
    
    if(scaleFactor == scaleFactor100PCT):
        s0.value(1)
        s1.value(1)
        
    if(scaleFactor == scaleFactor20PCT):
        s0.value(1)
        s1.value(0)
        
    if(scaleFactor == scaleFactor2PCT):
        s0.value(0)
        s1.value(1)


def getRedPulseWidth():
    # Get Red Pulse Width Value
    global isCaptureDone, RED_MIN, RED_MAX, pulseWidth, doCalibration
    s2.value(0)
    s3.value(0)
    redPulseWidth = 0
    avgPulseWidth = 0
    avgValue = 0
    avgPoints = 16
    
    
    for i in range(avgPoints):
        
        while not isCaptureDone:
            idle()
            
        status = disable_irq()
        
        redPulseWidth = pulseWidth
        
        avgPulseWidth = (avgPulseWidth + redPulseWidth)
        
        
        redValue = mapToRange(redPulseWidth, RED_MIN, RED_MAX, 255, 0)
        
        avgValue = avgValue + redValue
        
        isCaptureDone = False
        
        enable_irq(status)
    
    avgValue = avgValue // avgPoints
    
    if doCalibration:
            
        avgPulseWidth = avgPulseWidth // avgPoints
            
        print("ARWdth: {} ARVal: {}".format(avgPulseWidth, avgValue))
    
    return avgValue

def getGreenPulseWidth():
    
    # Get Green Pulse Width Value
    global isCaptureDone, GREEN_MIN, GREEN_MAX, pulseWidth, doCalibration
    s2.value(1)
    s3.value(1)
    greenPulseWidth = 0
    avgPulseWidth = 0
    avgValue = 0
    avgPoints = 16
    
    for i in range(avgPoints):
        
        while not isCaptureDone:
            idle()
            
        status = disable_irq()
        
        greenPulseWidth = pulseWidth
        avgPulseWidth = (avgPulseWidth + greenPulseWidth)
        
        greenValue = mapToRange(greenPulseWidth, GREEN_MIN, GREEN_MAX, 255, 0)
        
        avgValue = avgValue + greenValue
        
        isCaptureDone = False
        
        enable_irq(status)
    
    avgValue = avgValue // avgPoints
    
    if doCalibration:
        avgPulseWidth = avgPulseWidth // avgPoints
        print("AGWdth: {} AGVal: {}".format(avgPulseWidth, avgValue))
            
    return avgValue


def getBluePulseWidth():
    # Get Blue Pulse Width Value
    global isCaptureDone, BLUE_MIN, BLUE_MAX, pulseWidth, doCalibration
    s2.value(0)
    s3.value(1)
    bluePulseWidth = 0
    avgPulseWidth = 0
    avgValue = 0
    avgPoints = 16
    
    for i in range(avgPoints):
        
        while not isCaptureDone:
            idle()
            
        status = disable_irq()
        
        bluePulseWidth = pulseWidth
        
        avgPulseWidth = (avgPulseWidth + bluePulseWidth)
            
        blueValue = mapToRange(bluePulseWidth, BLUE_MIN, BLUE_MAX, 255, 0)
        
        avgValue = avgValue + blueValue
        
        isCaptureDone = False
        
        enable_irq(status)
    
    avgValue = avgValue // avgPoints
    
    if doCalibration:
        avgPulseWidth = avgPulseWidth // avgPoints
        print("ABWdth: {} ABVal: {}".format(avgPulseWidth, avgValue))
        
    return avgValue

s0, s1, s2, s3 = gpioSetup()

uart1 = uartSetup(115200)
uart1.write('Colour Sensor Read RGB Demo Program\r\n')

scaleFactor = 20 # 2 percent frequency scaling
setFreqScaling(s0, s1, scaleFactor)

while(True):
    
        r = getRedPulseWidth()
        
        g = getGreenPulseWidth()
      
        b = getBluePulseWidth()
        
        if not doCalibration:
            uart1.write(str(r) + "," +str(g)+ "," +str(b)+ "\r\n")
            
        time.sleep_ms(500)
        
        