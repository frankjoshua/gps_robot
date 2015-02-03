gps_robot
=========
Upload to the sever<br>
watch -n 10 rsync -avz ./gps.kml ec2-user@hostanme.com:~/

Download from the server<br>
watch -n 10 rsync -avzh ec2-user@hostname.com:~/*.kml ./
