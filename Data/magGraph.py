import matplotlib.pyplot as plt
import numpy as np

dataFile = open("cleanData.txt")	#Open data file
words = dataFile.read().split()		#Split into an array (spaces)

dataPts = len(words)/37

time = range(dataPts)			#Create time array with size = amount of data points
#acc = range(dataPts)			#Create acc array with size = amount of data points
accx = range(dataPts)			#Create acc in all 3 axies
accy = range(dataPts)
accz = range(dataPts)

for i in range(len(words)/37):			#Loop amount of data point times
	time[i] = float(words[i*37])/1000	#Set first var = the time converted to sec
						#Use this to add acc in all axies acc[i] = float(words[(i*37)+20]) + float(words[(i*37)+22]) + float(words[(i*37)+24])	#Set the second var to whatever you want, in this case all of the acc added up
	accx[i] = float(words[(i*37)+2])	#Add 20, 22, 24 for non calculated values
	accy[i] = float(words[(i*37)+4])	#Add 14, 16, 18 for the calculated values
	accz[i] = float(words[(i*37)+6])

linex, = plt.plot(time, accx, label="X")	#Plot time vs acc on the graph for x y and z axies
liney, = plt.plot(time, accy, label="Y")
linez, = plt.plot(time, accz, label="Z")

plt.legend(handles=[linex, liney, linez])	#Add a ledgend with labels for each graph

plt.xlabel('Time (s)')			#Label the x axis
plt.ylabel('Magnetic Force')		#Label the y axis
plt.title('Magnetic Force vs Time')	#Put a title on the grapg
plt.grid(True)				#Add grid
plt.show()				#Show the graph in a nice box

