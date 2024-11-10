from websockets.sync.client import connect

import argparse

import json

import time

import numpy as np

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



def main():
    
    x = []
    y = []
    
    plt.ion()
    
    parser = argparse.ArgumentParser(description='A script for updating Garai over the WiFi')
    
    parser.add_argument('address',metavar='addr',type=str,help="A robot address")
    # parser.add_argument('output',metavar='out',type=str,help="A path to the file where to store readings ( a CSV format )")
    
    args = parser.parse_args()
    
    address = args.address
    # filename = args.output
        
    # print("File with results: ",filename)
    
    graph = plt.plot(x,y)[0]
    
    try:
        with connect("ws://{}".format(address)) as websocket:
            
                
            while True:
                websocket.send("")
                
                msg:str = websocket.recv(timeout=25)
                
                readings = json.loads(msg)
                
                # print("T: {} s Left: {} speed Right: {} speed Yaw: {}".format(readings["T"],readings["left_motor_speed"],readings["right_motor_speed"],readings["yaw"]))
                
                # print("{};{};{};{}".format(readings["T"],readings["left_motor_speed"],readings["right_motor_speed"]/0.5,readings["yaw"]))
                
                id = np.argmin(readings["distances"])   
                
                x.append(float(readings["T"]))
                y.append(float(readings["left_motor_speed"]))             
                
                print(readings["T"]," target: ",readings["yaw"] - (id-2)*20," angle: ",readings["yaw"]," ",readings["distances"])
                print("Lowest yaw: ",readings["distances"][id])
                
                graph.remove()
                
                graph = plt.plot(x,y,"o",color="r")[0]
                
                # file.write(prepare_line(readings))
                
                time.sleep(0.001)
                plt.pause(0.001)
                
                if len(x) > 1000:
                    x = []
                    y = []
                                
    
    except Exception as e:
        print("Cannot connect to {}, with error {}".format(address,str(e)))


if __name__ == "__main__":
    main()