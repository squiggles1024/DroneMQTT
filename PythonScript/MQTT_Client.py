from paho.mqtt import client as mqtt
import matplotlib.pyplot as plt
from drawnow import *
import time
import struct
import keyboard
import queue
import cv2 as cv
import numpy as np

################################################################
# Variables Associated with MQTT Message Topics and Network info
################################################################
broker = "192.168.1.119"
port = 8883
MQTT_CLIENT_PUB_TOPIC01 = "Drone/Controls"

MQTT_CLIENT_SUB_TOPIC01 = "Env/Temperature"
MQTT_CLIENT_SUB_TOPIC02 = "Env/Humidity"
MQTT_CLIENT_SUB_TOPIC03 = "Env/Pressure"
MQTT_CLIENT_SUB_TOPIC04 = "Env/Light"
MQTT_CLIENT_SUB_TOPIC05 = "Env/Magnetic"
MQTT_CLIENT_SUB_TOPIC06 = "Drone/Roll"
MQTT_CLIENT_SUB_TOPIC07 = "Drone/Pitch"
MQTT_CLIENT_SUB_TOPIC08 = "Drone/Yaw"
MQTT_CLIENT_SUB_TOPIC09 = "Drone/RollRate"
MQTT_CLIENT_SUB_TOPIC10 = "Drone/PitchRate"
MQTT_CLIENT_SUB_TOPIC11 = "Env/Video"

MQTT_SubTopicList = [(MQTT_CLIENT_SUB_TOPIC01, 0),(MQTT_CLIENT_SUB_TOPIC02, 0),(MQTT_CLIENT_SUB_TOPIC03, 0),(MQTT_CLIENT_SUB_TOPIC04, 0),(MQTT_CLIENT_SUB_TOPIC05, 0),(MQTT_CLIENT_SUB_TOPIC06, 0),(MQTT_CLIENT_SUB_TOPIC07, 0),(MQTT_CLIENT_SUB_TOPIC08, 0),(MQTT_CLIENT_SUB_TOPIC09, 0),(MQTT_CLIENT_SUB_TOPIC10, 0),(MQTT_CLIENT_SUB_TOPIC11, 0)]
CertFileDirectory = 'C:\certs\ca.crt'
ClientName = "SurveillanceClient"

######################################
# Misc variables used for flow control
######################################
FirstMessageFlag = True                                  #Used to indicate that a message has been received from the Drone (The drone is 'on')
QuitKey          = 'esc'                                 #Press to Quit Button
SyncFrameFlag = False

##########################################################
#Image Parameters
##########################################################
X_Dim = 240
Y_Dim = 320
FrameBufferRGB565 = []
FrameBuffer = np.zeros((Y_Dim,X_Dim,3),dtype=np.uint8)
ImageChunkCounter = 0
ImageChunksPerImage = 10

##################################################
# Variables associated with keeping track of time
##################################################
TimeInitial = 0.0                                        #Used to Calculate "Delta T". Want to log data at T = 0, not T = Epoch time.
ElapsedTime = 0.0                                        #Variable to store elapsed time, used for data logging
ControllerTimeDelta = 0.20                               #The sample period of calculating
NextControllerTimeStamp = ControllerTimeDelta            #Keeps track of the Timestamp when the Next Controller sample should be published
PlotTimeDelta = 1.0                                      #How often the plot updates
ScrollTime = 11.0                                        #Time to start scrolling the plot
NextPlotTimeStamp = ScrollTime                           #Keeps track of when to update the plot next, Starts plot once queue fills
BulkDataPeriod = 0.1
BulkDataSamples = 10

##################################################
# Variables associated with keeping track of data
##################################################
TemperatureData = (0.0,)                                 #Tuple used to fetch temperature data from network
TemperatureDataBuffer = queue.Queue()                    #Queue used to store and plot temperature data
HumidityData = (0.0,)                                    #Tuple used to fetch humidity data from network
HumidityDataBuffer = queue.Queue()                       #Queue used to store and plot humidity data
PressureData = [(0.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0)]   #Tuple used to fetch pressure data from network
PressureDataBuffer = queue.Queue()                       #Queue used to store and plot pressure data
LightData = (0.0,)                                       #Tuple used to fetch light data from network
LightDataBuffer = queue.Queue()                          #Queue used to store and plot light data
MagneticXData = (0.0,)                                   #Tuple used to fetch magnetic data
MagneticYData = []                                     #Tuple used to fetch magnetic data
MagneticZData = []                                         #Tuple used to fetch magnetic data
MagneticXDataBuffer = queue.Queue()                      #Queue used to store and plot magneticX data
MagneticYDataBuffer = queue.Queue()                      #Queue used to store and plot magneticY data
MagneticZDataBuffer = queue.Queue()                      #Queue used to store and plot magneticZ data
TempTimeBuffer = queue.Queue()                            #Queue used to store TimeStamps of incoming messages
HumTimeBuffer = queue.Queue()                            #Queue used to store TimeStamps of incoming messages
LightTimeBuffer = queue.Queue()                            #Queue used to store TimeStamps of incoming messages
PresTimeBuffer = queue.Queue()                            #Queue used to store TimeStamps of incoming messages
MagTimeBuffer = queue.Queue()                            #Queue used to store TimeStamps of incoming messages


##########################################################
# Variables associated with Plot organization and labeling
##########################################################
NumberOfPlots = 7
TemperatureFigure = 1
TemperatureFigureLabel = "Temperature (C)"
HumidityFigure = 2
HumidityFigureLabel = "Humidity (Relative %)"
PressureFigure = 3
PressureFigureLabel = "Pressure (hPA)"
LightFigure = 4
LightFigureLabel = "Light (Lux)"
MagneticXFigure = 5
MagneticXFigureLabel = "Magnetic Field X (mg)"
MagneticYFigure = 6
MagneticYFigureLabel = "Magnetic Field Y (mg)"
MagneticZFigure = 7
MagneticZFigureLabel = "Magnetic Field Z (mg)"
TimeBufferLabel = "Time (s)"

##########################################################
# Function to subscribe to all Topics
##########################################################
def SubscribeAll(Client):
    Client.subscribe(MQTT_SubTopicList)

##########################################################
# Function to subscribe to all Topics
##########################################################
def UnsubscribeAll(Client):
    Client.unsubscribe(MQTT_SubTopicList)


##################################################
#@Brief: Functionm that shows matplotlib plot data
##################################################
def PlotData():
    plt.xlabel(TimeBufferLabel)

    plt.subplot(1,NumberOfPlots,TemperatureFigure)
    plt.title(TemperatureFigureLabel)
    Bound = max(TemperatureDataBuffer.queue)*0.1
    plt.ylim(min(TemperatureDataBuffer.queue) - abs(Bound), max(TemperatureDataBuffer.queue) + abs(Bound))
    plt.plot(TempTimeBuffer.queue,TemperatureDataBuffer.queue, )

    plt.subplot(1,NumberOfPlots,HumidityFigure)
    plt.title(HumidityFigureLabel)
    Bound = max(HumidityDataBuffer.queue)*0.1
    plt.ylim(min(HumidityDataBuffer.queue) - Bound, max(HumidityDataBuffer.queue) + Bound)
    plt.plot(HumTimeBuffer.queue,HumidityDataBuffer.queue)

    plt.subplot(1,NumberOfPlots,PressureFigure)
    plt.title(PressureFigureLabel)
    Bound = max(PressureDataBuffer.queue)*0.1
    plt.ylim(min(PressureDataBuffer.queue) - Bound, max(PressureDataBuffer.queue) + Bound)
    plt.plot(PresTimeBuffer.queue,PressureDataBuffer.queue)

    plt.subplot(1,NumberOfPlots,LightFigure)
    plt.title(LightFigureLabel)
    Bound = max(LightDataBuffer.queue)*0.1
    plt.ylim(min(LightDataBuffer.queue) - Bound, max(LightDataBuffer.queue) + Bound)
    plt.plot(LightTimeBuffer.queue,LightDataBuffer.queue)

    plt.subplot(1,NumberOfPlots,MagneticXFigure)
    plt.title(MagneticXFigureLabel)
    Bound = max(MagneticXDataBuffer.queue)*0.1
    plt.ylim(min(MagneticXDataBuffer.queue) - Bound, max(MagneticXDataBuffer.queue) + Bound)
    plt.plot(MagTimeBuffer.queue,MagneticXDataBuffer.queue)

    plt.subplot(1,NumberOfPlots,MagneticYFigure)
    plt.title(MagneticYFigureLabel)
    Bound = max(MagneticYDataBuffer.queue)*0.1
    plt.ylim(min(MagneticYDataBuffer.queue) - Bound, max(MagneticYDataBuffer.queue) + Bound)
    plt.plot(MagTimeBuffer.queue,MagneticYDataBuffer.queue)

    plt.subplot(1,NumberOfPlots,MagneticZFigure)
    plt.title(MagneticZFigureLabel)
    Bound = max(MagneticZDataBuffer.queue)*0.1
    plt.ylim(min(MagneticZDataBuffer.queue) - Bound, max(MagneticZDataBuffer.queue) + Bound)
    plt.plot(MagTimeBuffer.queue,MagneticZDataBuffer.queue)

################################################################
#@Brief: Display Image
################################################################
def DisplayImage():
    global FrameBufferRGB565
    global FrameBuffer
    i = 0
    for x in range(X_Dim):
        for y in range(Y_Dim):
            Pixel = FrameBufferRGB565[i]
            i = i + 1
            #FrameBuffer[y][x][0] = (Pixel&0x001F) << 3 
            #FrameBuffer[y][x][1] = (Pixel&0x07E0) >> 3
            #FrameBuffer[y][x][2] = (Pixel&0xF800) >> 8
            #Rotate frame 90 degrees
            FrameBuffer[Y_Dim - y - 1][x][0] = (Pixel&0x001F) << 3 
            FrameBuffer[Y_Dim - y - 1][x][1] = (Pixel&0x07E0) >> 3
            FrameBuffer[Y_Dim - y - 1][x][2] = (Pixel&0xF800) >> 8
    cv.imshow("Video", FrameBuffer)
    cv.waitKey(1)
    


################################################################
#@Brief: Method that runs everytime client subscribes to a topic
################################################################
def on_subscribe(client, userdata, mid, granted_qos):
    print('Subscribed for m' + str(mid))

################################################################
#@Brief: Method that runs everytime client subscribes to a topic
################################################################
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+ str(rc))

#########################################################
#@Brief: Method that runs everytime an MQTT event happens (Current unused)
#########################################################
def on_log(client, userdata, level, buf):
    print("log: ", buf)

#################################################
#@Brief: Method that runs when client disconnects
#################################################
def on_disconnect(client, userdata, rc):
    print("Client Disconnected at time: ", ElapsedTime)

#############################################################
#@Brief: Method that runs everytime client recieves a message
#############################################################
def on_message(client, userdata, message):                                         
    global ElapsedTime
    global PlotTimeDelta
    global TimeInitial
    global FirstMessageFlag
    global TemperatureData
    global HumidityData
    global PressureData
    global LightData
    global FrameBuffer
    global SyncFrameFlag
    global ImageChunkCounter

    #Initialize Time at first message and return
    if(FirstMessageFlag == True):
        FirstMessageFlag = False
        TimeInitial = time.time()
        return

    #Discard all data captured in the first second
    if(ElapsedTime < 1.0):
        return
    
    #Don't start handling Video data until 1.0s has passed AND a non-video packet has arrived
    if(message.topic != MQTT_CLIENT_SUB_TOPIC11):
        SyncFrameFlag = True

    print(ElapsedTime)
    #Handle Video Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC11):
        if(SyncFrameFlag):
            ImageChunk = struct.iter_unpack('H', message.payload)
            for Chunk in ImageChunk:
                FrameBufferRGB565.append(Chunk[0])
            ImageChunkCounter = ImageChunkCounter + 1
            if(ImageChunkCounter == ImageChunksPerImage):
                ImageChunkCounter = 0
                DisplayImage()
                FrameBufferRGB565.clear()
            return

    #Temperature Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC01):
        TemperatureData = struct.unpack('f',message.payload)
        if(ElapsedTime > 11.0):
            TemperatureDataBuffer.get()
            TempTimeBuffer.get()
        TemperatureDataBuffer.put(TemperatureData[0])
        TempTimeBuffer.put(ElapsedTime)
        return
    
    #Humidity Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC02):
        HumidityData = struct.unpack('f',message.payload)
        if(ElapsedTime > 11.0):
            HumidityDataBuffer.get()
            HumTimeBuffer.get()
        HumidityDataBuffer.put(HumidityData[0])
        HumTimeBuffer.put(ElapsedTime)
        return

    #Light Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC04):
        LightData = struct.unpack('f',message.payload)
        if(ElapsedTime > 11.0):
            LightDataBuffer.get()
            LightTimeBuffer.get()
        LightDataBuffer.put(LightData[0])
        LightTimeBuffer.put(ElapsedTime)
        return
    
    #Pressure Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC03):
        PressureData = struct.unpack('10f',message.payload)
        i = 0
        while i < BulkDataSamples:
            if(ElapsedTime > 11.0):
                PressureDataBuffer.get()
                PresTimeBuffer.get()
            PressureDataBuffer.put(PressureData[i])
            PresTimeBuffer.put(ElapsedTime - BulkDataPeriod*(BulkDataSamples - i))
            i = i + 1
        return

    #Magnetic Data
    if (message.topic == MQTT_CLIENT_SUB_TOPIC05):
        MagneticXData = struct.unpack_from('10f',message.payload,0)
        MagneticYData = struct.unpack_from('10f', message.payload,10*4) 
        MagneticZData = struct.unpack_from('10f', message.payload,20*4)
        i = 0
        while i < BulkDataSamples:
            if(ElapsedTime > 11.0):
                MagneticXDataBuffer.get()
                MagneticYDataBuffer.get()
                MagneticZDataBuffer.get()
                MagTimeBuffer.get()
            MagneticXDataBuffer.put(MagneticXData[i])
            MagneticYDataBuffer.put(MagneticYData[i])
            MagneticZDataBuffer.put(MagneticZData[i])
            MagTimeBuffer.put(ElapsedTime - BulkDataPeriod*(BulkDataSamples - i))
            i = i + 1
        return


    #if(ElapsedTime > NextPlotTimeStamp):
        #UnsubscribeAll(client)
        #drawnow(PlotData)
        #NextPlotTimeStamp = ElapsedTime + PlotTimeDelta
        #SubscribeAll(client)

#############################################################################
#@Brief: Function Updates elapsed time if the first message has been received
#############################################################################
def UpdateTime():
    global ElapsedTime
    if(FirstMessageFlag == False):
        ElapsedTime = (time.time() - TimeInitial)

#############################################################################
#@Brief: Function Sends Controller data
#############################################################################
def SendControllerData(Client):
    global NextControllerTimeStamp
    if(ElapsedTime > NextControllerTimeStamp):
        Client.publish(MQTT_CLIENT_PUB_TOPIC01, ControllerState, qos=1)
        NextControllerTimeStamp = ElapsedTime + ControllerTimeDelta

#############################################################################
#@Brief: Function Sends Controller data
#############################################################################
def UpdatePlot():
    global NextPlotTimeStamp
    if(ElapsedTime > NextPlotTimeStamp):
        drawnow(PlotData)
        NextPlotTimeStamp = ElapsedTime + PlotTimeDelta

##########################################################################
#"Main" Begins Here
##########################################################################
ControllerState = struct.pack("B",98)                         #Dummy Data for STM32 debug/test
SurvClient = mqtt.Client(ClientName,clean_session=True)
SurvClient.on_message = on_message
SurvClient.on_connect = on_connect
SurvClient.on_disconnect = on_disconnect
SurvClient.on_subscribe = on_subscribe
#SurvClient.on_log = on_log
SurvClient.tls_set(ca_certs=CertFileDirectory)
SurvClient.connect(broker,port)
SubscribeAll(SurvClient)
time.sleep(3)
SurvClient.loop_start()
while True:
    if keyboard.is_pressed(QuitKey):
        break
    UpdateTime()
    UpdatePlot()
    SendControllerData(SurvClient)
SurvClient.disconnect()