from websockets.sync.client import connect

import argparse

import json
import numpy as np

import time

import matplotlib.pyplot as plt

def prepare_line(json:dict,end="\n"):
    
    line:str = ""
    
    for item in list(json.values())[:-1]:
        
        if type(item) is dict:
            line+= prepare_line(item,'')+";"
        else:
            line += str(item)+";"
    
    item = list(json.values())[-1]
    
    if type(item) is dict:
        line+= prepare_line(item,'')
    else:
        line += str(item)
    
    line += end
    
    return line


x = []
y = []

x_off = 0 #0.76825573 #0.969241535 #2.74011
y_off = 0 #1.552019993 #1.4565894265 #2.8739

def main():
    global x
    global y
    parser = argparse.ArgumentParser(description='A script for updating Garai over the WiFi')
    
    parser.add_argument('address',metavar='addr',type=str,help="A robot address")
    parser.add_argument('output',metavar='out',type=str,help="A path to the file where to store readings ( a CSV format )")
    
    args = parser.parse_args()
    
    x_buff = []
    y_buff = []
    
    address = args.address
    filename = args.output
        
    print("File with results: ",filename)
    
    plt.ion()
    
    graph = plt.plot(x,y)[0]
    
    stdev = 0.0125
    
    V = stdev*(1/100)*0.01
    W = stdev*stdev
    
    x0=0
    P0 = 1
    x1 = x0
    xpost = x0
    xpri = x0
    Ppost = P0
    Ppri = P0
    
    y0 = 0
    Py0 = 1
    y1 = y0
    ypost = y0
    ypri = y0
    Pypost = Py0
    Pypri = Py0
    
    first = True

    
    try:
        with connect("ws://{}".format(address)) as websocket:
            
            with open(filename,"w") as file:
                
                while True:
                    websocket.send("")
                    
                    msg:str = websocket.recv(timeout=25)
                    
                    readings = json.loads(msg)
                    
                    x_raw = readings["raw_magentrometer"]["x"] - x_off
                    y_raw = readings["raw_magentrometer"]["y"] - y_off
                    
                    # if first:
                    
                    #     xpri = xpost
                        
                    #     Ppri = Ppost + V
                    #     E = x_raw - xpri
                        
                    #     S = Ppri + W
                        
                    #     K = Ppri/S
                        
                    #     xpost = xpri + K*E
                    #     Ppost = Ppri - S*K*K
                        
                    #     ypri = ypost
                        
                    #     Pypri = Pypost + V
                    #     E = y_raw - ypri
                        
                    #     S = Pypri + W
                        
                    #     K = Ppri/S
                        
                    #     ypost = ypri + K*E
                    #     Pypost = Pypri - S*K*K
                    # else:
                    #     first = False
                        
                    #     xpost = x_raw
                    #     xpri = x_raw
                        
                    #     ypost = y_raw
                    #     ypri = y_raw
                    
                    text:str = "{};{};{}".format(x_raw,y_raw,readings["raw_magentrometer"]["z"])
                    
                    x.append(x_raw)
                    y.append(y_raw)
                    
                    # if len(x)>0 and len(y)>0:
                    #     std = np.std([x_raw,x[-1]])
                        
                    #     if std < stdev:
                    #         x_raw = x[-1]
                            
                    #     std = np.std([y_raw,y[-1]])
                        
                    #     if std < stdev:
                    #         y_raw = y[-1]
                    
                    # if len(x_buff)>10:
                        
                    #     x.append(np.mean(x_buff))
                    #     x_buff = []
                    # else:
                    #     x_buff.append(x_raw)
                        
                    # if len(y_buff)>10:
                        
                    #     y.append(np.mean(y_buff))
                    #     y_buff = []
                    # else:
                    #     y_buff.append(y_raw)
                    
                    
                    # x.append(x_raw)
                        
                    
                    # y.append(y_raw)
                    # y.append(0.75*y_raw+y[-1]*0.25)
                                            
                    graph.remove()
                    
                    graph = plt.plot(x,y,"o",color="r")[0]
                    
                    # angle = np.arctan2(readings["raw_magentrometer"]["y"],readings["raw_magentrometer"]["x"])*(180.0/np.pi)
                    
                    print(text)
                    # print("X: {} Y: {} X_STD: {} Y_STD: {}".format(np.mean(x),np.mean(y),np.std(x),np.std(y)))
                    # print(np.arctan2(y[-1],x[-1])*(180.0/np.pi))
                    file.write(text+'\n')
                    
                    # input()
                    time.sleep(0.01)
                    plt.pause(0.01)
                    
                    if len(x)>1000 and len(y)>1000:
                        print("Mean: X: {} Y: {}".format(np.mean(x),np.mean(y)))
                        x = []
                        y = []
                        exit(0)
                                
    
    except Exception as e:
        print("Cannot connect to {}, with error {}".format(address,str(e)))


if __name__ == "__main__":
    main()