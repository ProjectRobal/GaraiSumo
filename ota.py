from websockets.sync.client import connect

import argparse

import os

def prepare_hello_frame(file_size:int):
    
    arr = bytearray()
    
    arr.append(0x00)
    
    arr.extend(file_size.to_bytes(4,'little'))
    
    return arr

def prepare_flash_frame(id:int,data:bytes):
    
    arr = bytearray()
    
    arr.append(0x01)
    
    arr.extend(id.to_bytes(4,'little'))
    
    if len(data)<4096:
        to_fill:int = 4096 - len(data)
        
        arr.extend(bytes([0x00]*to_fill))
    
    arr.extend(data)
    
    return arr
    

def main():
    parser = argparse.ArgumentParser(description='A script for updating Garai over the WiFi')
    
    parser.add_argument('address',metavar='addr',type=str,help="A robot address")
    parser.add_argument('directory',metavar='dict',type=str,help="A directory where binary file is located")
    parser.add_argument('name',metavar='n',type=str,help="A name of the binary file with program's code")
    
    args = parser.parse_args()
    
    address = args.address
    directory = args.directory
    filename = directory+"/"+args.name + ".bin"
    
    if not os.path.exists(directory):
        print("Directroy doesn't exits")
        exit(-1)
        
    print("Found a file: ",filename)
        
    file_size:int = os.path.getsize(filename)
    
    print("File size: ",file_size," B")

    chunk_count:int = int(file_size/4096)
    
    print("Chunk size: ",chunk_count)
    
    try:
        with connect("ws://{}".format(address)) as websocket:
            
            with open(filename,"rb") as file:
                
                websocket.send(prepare_hello_frame(chunk_count))
                
                msg:str = websocket.recv(timeout=25)
                
                if msg != b"OK":
                    print("Websocket timeout")
                    exit(-2)
                    
                chunk_id = 0
                
                while True:
                    chunk = file.read(4096)
                    
                    if chunk is None:
                        break
                    
                    print("Flashing chunk {} out of {} chunks".format(chunk_id,chunk_count))
                    websocket.send(prepare_flash_frame(chunk_id,chunk))
                    
                    chunk_id += 1
                    
                    msg:str = websocket.recv(timeout=25)
                    
                    # print("Recv: ",msg)
                    
                    if msg == b'ERR':
                        print("Error during flashing!")
                        exit(-4)
                
                    if msg != b'OK' and msg != b'END':
                        print("Websocket timeout during flashing")
                        exit(-3)
                        
                    if msg == b'END':
                        break
                        
        print("Finished flashing!")
                
    
    except Exception as e:
        print("Cannot connect to {}, with error {}".format(address,str(e)))


if __name__ == "__main__":
    main()