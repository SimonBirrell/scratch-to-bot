scratch-to-bot
==============

This software creates an interface between MIT's Scratch and Dawn Robotic's Camera Bot. It leverages the scratchpy library that reads and writes to Scratch, and Dawn Robotic's py_websockets_bot.py that provides an interface to the websockets server on the bot itself.

My contribution is the Python script that lives in the middle: scratch_to_bot.py. It was written in a hurry to get a robot working for my five and a half year old niece for Christmas. It's also my first Python program, so please be indulgent.

I've tested the software on Mac OS X and also on the Raspberry Pi. In both cases they can control the robot remotely.

Installing software on Scarlett Pi
----------------------------------

1. Copy all the code to a new directory under /home

2. Add execute permission to the Python script

     chmod +x scratch_to_bot.py

3. Adjust the hostname in scratch_to_bot.py

Launching the software
----------------------

1. Follow the instructions to install [py_websockets_bot](https://bitbucket.org/DawnRobotics/py_websockets_bot) and [scratchpy](https://github.com/pilliq/scratchpy).

2. Ensure that the robot is switched on and that you are on the same WiFi network.

3. Launch Scratch

4. Launch the Python script from a terminal window with

     python scratch_to_bot.py

5. In Scratch, right click on one of the sensor blocks and select "Enable Remote Sensor Communications"

Programming the robot
---------------------

Program the robot using the broadcast block in Scratch. Inside the broadcast block, specify a command such as

robot forward 1

In the configuration of my robot I have an ultrasonic sensor to measure distance to the nearest obstacle. This appears as a sensor called "distance" inside Scratch and ranges from 0mm to 400mm.

Robot Commands
--------------

    robot forward 1
    robot forward 2
etc.
Move the robot forwards a distance then stop.

    robot backward 1
    robot backward 2
etc.
Move the robot backwards a distance then stop.

    robot turn left
    robot turn right
    robot turn -90
    robot turn +90
Turn the robot to face a new direction.

    robot camera on
    robot camera off
This launches VLC and streams video from the robot. It works fine on the Mac but siezes up on the Pi. I think the stream is too heavy for the Pi to handle at the same time as Scratch and the Python script. I'll have a look at fixing this.

    robot stop
This stops all motors on the bot. In contrast to the above commands, this interrupts the robot's current activities. The normal movement commands queue up, so if scratch sends ten "robot forward 1" then they will get queued up and executed one after another. Having "robot stop" interrupt and clear the queue allows you to do a "repeat until" loop that stops as soon as, say, a distance condition is met. In an ideal world, Scratch broadcast commands would block, but I'm not sure if that is possible.

    robot neck -90
    robot neck 90
Pan the robot "neck".

    robot head -90
    robot head 90
Tilt the robot head.

Concluding Thoughts
-------------------
This is a first cut at a solution. I'd love to have feedback. I'm sure my Python sucks.

Thanks
------
Thanks to "Pilliq" for the Scratch library and Alan at Dawn Robotics for py_websockets_bot and for the robot itself.

- https://github.com/pilliq/scratchpy
- http://www.dawnrobotics.co.uk/raspberry-pi-camera-robot-chassis-bundle/

License
-------

MIT sounds good.














