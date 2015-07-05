gps_robot
=========

This is the code I used for Robomo's 2015 Magellan Competition<br>
http://robomo.com/competitions/2015-magellan-competition/<br>
<br>
See this https://forums.adafruit.com/viewtopic.php?f=19&t=75497 for information on calibrating the BNO055 Sensor to work as a compass<br>
/arduino_libs/ holds the libraries used by the sketch<br>
/gps_arduino/ is where the main sketch is located<br>
/gegpsd.py is the python script that reads raw gps data and turns it into a kml file for Google Earth<br>

This is how I upload the kml file to my server:<br>

Upload to the sever<br>
watch -n 10 rsync -avz ./gps.kml ec2-user@hostanme.com:~/

Download from the server<br>
watch -n 10 rsync -avzh ec2-user@hostname.com:~/*.kml ./
