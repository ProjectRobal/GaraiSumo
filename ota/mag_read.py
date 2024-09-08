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

def main():
    parser = argparse.ArgumentParser(description='A script for updating Garai over the WiFi')
    
    parser.add_argument('address',metavar='addr',type=str,help="A robot address")
    parser.add_argument('output',metavar='out',type=str,help="A path to the file where to store readings ( a CSV format )")
    
    args = parser.parse_args()
    
    address = args.address
    filename = args.output
        
    print("File with results: ",filename)
    
    plt.ion()
    
    graph = plt.plot(x,y)[0]
    
    yaw = []
    
    mean_angle = 0
    
    try:
        with connect("ws://{}".format(address)) as websocket:
            
            with open(filename,"w") as file:
                
                while True:
                    websocket.send("")
                    
                    msg:str = websocket.recv(timeout=25)
                    
                    readings = json.loads(msg)
                    
                    text:str = "{};{};{}".format(readings["raw_magentrometer"]["x"],readings["raw_magentrometer"]["y"],readings["raw_magentrometer"]["z"])
                    
                    x.append(readings["raw_magentrometer"]["x"])
                    y.append(readings["raw_magentrometer"]["y"])
                    
                    graph.remove()
                    
                    graph = plt.plot(x,y,"o",color="r")[0]
                    
                    # angle = np.arctan2(readings["raw_magentrometer"]["y"],readings["raw_magentrometer"]["x"])*(180.0/np.pi)
                    
                    print(text)
                    file.write(text+'\n')
                    
                    # input()
                    time.sleep(0.01)
                    plt.pause(0.01)
                                
    
    except Exception as e:
        print("Cannot connect to {}, with error {}".format(address,str(e)))


if __name__ == "__main__":
    main()