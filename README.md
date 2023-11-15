# Tactile_control_Ale
## Haptic finger connection
## Haptic finger:

Move now on the PC of the robot.
Connect the micro-usb to the port close to HTMI port (the one in the middle) of he raspberryppi device.
In the wired connection setting of the sensor (Linux Network settings), set raspberryppi IPV6 to "Disable" and the IPV4 to "Linnk-Local".
Open now different terminals and do the folowing instructions.

### firewall disabling
If a firewall is enabled you should disable it in one of the following ways:
```
sudo ufw disable #disable permanently the firewall
```
or temporarly (you have to do the procedure again if you restart the pc):
```
sudo iptables -L
sudo iptables -F   # Flush all rules
sudo iptables -P INPUT ACCEPT   # Allow all incoming traffic
sudo iptables -P FORWARD ACCEPT # Allow all forwarded traffic
```

### Terminal 1 -  client terminal 
```
ssh pi@raspberrypi.local   # (password:  2880)
sudo nano client.py
```

### Terminal 2
Get the raspberrypi ip adress:
```
hostname -I
```
### Terminal 1
Edit the raspberrypi ip adress which you already got in the "hostmame -I" output, it should start with "169.254." edit that ip adress then "ctr+o" to save and "ctr+x" to exit

### Terminal 3 - server terminal
Locate the server_sersor.py then edit the raspberrypi ip adress (same as client.py)
```
sudo nano server.py
```

### Terminal 4
```
roscore
```

### Terminal 3
Now you can run the server first by:
```
python3 server.py
```

### Terminal 1
And run the client on the pi terminal
```
python3 client.py
```

Everything should be working alright now and you can check by rostopic echo that if the sensor image is being published properly.
```
rostopic list  # the topic published should be /fing_camera/color/image_raw
rostopic echo .............
```

For seeeing the image you can also do:
```
rqt_image_viewer #and select the topic to print
```
Troubleshoot:
- If you cannot ssh (or cannot ping raspberrypi.local), sometime a restart or reconnecting the sensor will solve the prblem.