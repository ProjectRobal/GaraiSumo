from websockets.sync.client import connect

import argparse

import json

import time


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
    parser = argparse.ArgumentParser(description='A script for updating Garai over the WiFi')
    
    parser.add_argument('address',metavar='addr',type=str,help="A robot address")
    parser.add_argument('output',metavar='out',type=str,help="A path to the file where to store readings ( a CSV format )")
    
    args = parser.parse_args()
    
    address = args.address
    filename = args.output
        
    print("File with results: ",filename)
    
    try:
        with connect("ws://{}".format(address)) as websocket:
            
            with open(filename,"w") as file:
                
                while True:
                    websocket.send("")
                    
                    msg:str = websocket.recv(timeout=25)
                    
                    readings = json.loads(msg)
                    
                    text:str = "{};{};{}".format(readings["raw_magentrometer"]["x"],readings["raw_magentrometer"]["y"],readings["raw_magentrometer"]["z"])
                    
                    print(text)
                    file.write(text+'\n')
                    
                    input()
                                
    
    except Exception as e:
        print("Cannot connect to {}, with error {}".format(address,str(e)))


if __name__ == "__main__":
    main()