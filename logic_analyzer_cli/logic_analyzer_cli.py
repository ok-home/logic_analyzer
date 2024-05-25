import serial
import serial.tools.list_ports
import json
import sys

cfgFileName = "la_cfg.json"
laConfig = {}

def portList():
# get available port list
    ports = list(serial.tools.list_ports.comports())
# output available port list
    print("Available ports")
    for port in ports:
        print(f"Port: {port.device}")
        print(f"Description: {port.description}")
        print(f"Manufacturer: {port.manufacturer}")

def defaultCfgToFile():
    global cfgFileName
    laDefaultCfg = {
        "PyCfg":{
            "dataFile":"laRowBin.bin",
            "PortName":"/dev/ttyACM1",
            "PortSpeed":"921600",
            "PortTimeout":"120" 
        },
        "EspCfg":{
            "pin0":"-1",
            "pin1":"-1",
            "pin2":"-1",
            "pin3":"-1",
            "pin4":"-1",
            "pin5":"-1",
            "pin6":"-1",
            "pin7":"-1",
            "pin8":"-1",
            "pin9":"-1",
            "pin10":"-1",
            "pin11":"-1",
            "pin12":"-1",
            "pin13":"-1",
            "pin14":"-1",
            "pin15":"-1",
            "trg":"-1",
            "edg":"0",
            "smp":"1000",
            "clk":"1000000",
            "tmo":"20",
            "chn":"16",
            "ram":"0"
        }
    }
    with open(cfgFileName,"w") as f:
        json.dump(laDefaultCfg,f,indent=1)

def readCfgFromFile():
    global laConfig
    global cfgFileName
    try:
        with open(cfgFileName,"r") as f:
            laConfig = json.load(f)
    except:
        defaultCfgToFile()
        with open(cfgFileName,"r") as f:
            laConfig = json.load(f)

def laCfgToEsp():  
    global ser
    global laConfig 
    for key in laConfig["EspCfg"]:
        str = json.dumps({"key":key,"msg":laConfig["EspCfg"][key]})+"\n"
        ser.write(bytes(str,'utf-8'))

def getAvailableCfgFromEsp():
    ser.write(bytes("getcfg\n",'utf-8'))
    print("-------------------------------------------")
    print(ser.readline().decode("utf-8").replace("\n",""))
    print(ser.readline().decode("utf-8").replace("\n",""))
    print(ser.readline().decode("utf-8").replace("\n",""))
    print(ser.readline().decode("utf-8").replace("\n",""))
    print(ser.readline().decode("utf-8").replace("\n",""))
    #print("-------------------------------------------")
    
def readLaDataFromEsp():
    global ser
    global laConfig
    ser.reset_input_buffer()
    ser.write(bytes("endcfg\n",'utf-8'))
    param = {}

    while 1:
        cmd = ser.readline().decode("utf-8")
        match cmd: 
            case "Start samples transfer\n":
                print("Start samples transfer")
                if param['chn'] == '16':
                    cnt = int(param['smp'])*2
                else :
                    cnt = int(param['smp'])
                data = ser.read(cnt)
                with open(laConfig["PyCfg"]["dataFile"],"wb") as f:
                    f.write(data)
                print("Samples transferred to file "+laConfig["PyCfg"]["dataFile"])
                print("Samples transferred "+str(len(data))+" of " +param['smp']+", sample rate="+param['clk']+" channel="+param['chn'])
                break
            case x if "{" in x:
                param.update(json.loads(cmd))
            case "Start logic analyzer OK\n":
                print("Start logic analyzer OK")
            case "Samples transfer done\n":
                print("Samples transfer done")
                break
            case "Start logic analyzer error\n":
                print("Error - check logic analyzer param in cfg.json")
                break
            case "Error - callback timeout deteсted\n":
                print("Error: logic analyzer timeout")
                break

    getAvailableCfgFromEsp()

def main():
    global cfgFileName
    global ser
    global laConfig
    
    if len(sys.argv) == 1: # use default cfg filename
        print("Use default cfg file la_cfg.json")
        cfgFileName = "la_cfg.json"
    else:
        cfgFileName = sys.argv[1]
    print("-------------------------------------------")
    readCfgFromFile()
    try:
        ser = serial.Serial(laConfig["PyCfg"]["PortName"], int(laConfig["PyCfg"]["PortSpeed"]),timeout=float(laConfig["PyCfg"]["PortTimeout"]))
        laCfgToEsp()
        readLaDataFromEsp()
        ser.close()
    except:
        print(f"Error open serial port {laConfig['PyCfg']['PortName']}")
        print("-------------------------------------------")
        portList()
    print("-------------------------------------------")

main()
print("exit")