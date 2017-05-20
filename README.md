Middleton High School RFS 2017 Data
====================================
On May 13, 2017 the Middleton High School Rocket Team launched a class 2 rocket with a payload consisting of a microcontroller and sensors for the [Rockets For Schools](http://www.rockets4schools.org/) competition. The Arduino for the project was connected to a 9dof sensor stick from [Sparkfun (SEN-13944)](https://www.sparkfun.com/products/13944) via i2c and collected acceleration, magnetic fields, and rotation (gyro) in all 3 axies. All of the data was saved to a SD card which was later uploaded to this project as a txt file.

This repo contains a python script that allows you to graph values that the rocket collected during flight. It uses [Matplotlib](https://matplotlib.org/) to do the graphing.

The cleanData.txt file contains the data from the rocket launch minus the commas. The testData.txt file contains a few test points useful for checking to make sure you are using the right values. Also included is the raw data file which includes data from before and after the launch

Launch Details
--------------
The launch started at about 2467305 ms and ended around 2571305 ms for a total flight time of about 1:44.

Here are a few graphs that I created
------------------------------------
Acceleration Graph
![Acceleration Graph](/Data/img/accGraph.png)

Gyro Graph
![Gyro Graph](/Data/img/gyroGraph.png)

Mag Graph
![Mag Graph](/Data/img/magGraph.png)
