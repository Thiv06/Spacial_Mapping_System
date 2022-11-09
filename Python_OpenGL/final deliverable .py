

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.
import sys

sys.path.append(r"C:\Users\Thivagar\AppData\Local\Programs\Python\Python38")
import serial
import numpy as np
import open3d as o3d
import math
import time 

s=serial.Serial("COM6", 115200, timeout=10) #designated port, baud rate, timeout
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())
# recieve 10 measurements from UART of MCU

distanceValues =[]
j=0 
scanNum=0
 #read the entire line, not each character 

for i in range(9):
    x = s.readline()
    print(x.decode())

index=0;
j=0

for i in range(3):
    print("Beginning Scan...")
    time.sleep(5)
    print("Scan Number: "+ str(i+1)) 
    while j<8:
       y=s.readline()
       measurement=int(float(y))
       if measurement<10000:
           distanceValues.insert(index,measurement) #adding measurements to our array
           print(distanceValues[index]);
           index=index+1
           j=j+1
       else:
           print("Scan Ended Abruptly. Dropping Measurements")
           removeM=measurement/10000
           removeM=int(removeM)
           j=0
           print(distanceValues)
           del distanceValues[-removeM:]
           index=index-removeM
           print(distanceValues)
           time.sleep(5)
           print("\nRedo Scan: "+ str(i+1))
           #y=s.readline()
           time.sleep(10)
           print("Beginning...")
    j=0
          
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()

yDistance=[]
zDistance=[]

pi=math.pi
newIndx=0

for i in range(10):
    for j in range(8):
        yDistance.insert(newIndx,(distanceValues[newIndx]*math.cos(pi/4*j)))
        zDistance.insert(newIndx,(distanceValues[newIndx]*math.sin(pi/4*j)))

if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data 
    f = open("testrun.xyz", "w")    #create a new file for writing 
    
    #Test data: Lets make a rectangular prism as a point cloud in XYZ format
    #   A simple prism would only require 8 vertices, however we
    #   will sample the prism along its x-axis a total of 10x
    #   4 vertices repeated 10x = 40 vertices
    #   This for-loop generates our test data in xyz format
    for x in range(10):
        f.write('{} {} {}\n'.format((x*1300),yDistance[x],zDistance[x]))    #write x,0,0 (xyz) to file as p1
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+1],zDistance[x+1]))    #write x,0,1 (xyz) to file as p2
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+2],zDistance[x+2]))   #write x,1,1 (xyz) to file as p3
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+3],zDistance[x+3]))    #write x,0,0 (xyz) to file as p1
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+4],zDistance[x+4]))    #write x,0,1 (xyz) to file as p2
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+5],zDistance[x+5])) 
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+6],zDistance[x+6]))    #write x,0,0 (xyz) to file as p1
        f.write('{} {} {}\n'.format((x*1300),yDistance[x+7],zDistance[x+7]))    #write x,0,1 (xyz) to file as p2
    f.close()   #there should now be a file containing 80 vertex coordinates                               
    
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("testrun.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    #OK, good, but not great, lets add some lines to connect the vertices
    #   For creating a lineset we will need to tell the packahe which vertices need connected
    #   Remember each vertex actually contains one x,y,z coordinate

    #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,80):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,80,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x]])
   

    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,70,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+8]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+9]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+10]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+11]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+12]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+13]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+14]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+15]])

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])


        


                            


    
