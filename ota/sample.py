import numpy as np


x_buff = []
y_buff = []

with open("./mag_out.csv","w") as output:
    with open("./mag.csv","r") as file:
        for line in file.readlines():
            
            values = line.split(";")
            
            x_buff.append(float(values[0]) - 0.668624997138977)
            y_buff.append(float(values[1]) - 1.14145833253861)
            
            if len(x_buff)>=20 and len(y_buff)>=20:
                x = np.mean(x_buff)
                y = np.mean(y_buff)
                x_buff.clear()
                y_buff.clear()
                
                output.write("{};{}\n".format(x,y))
        
        