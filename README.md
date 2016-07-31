#RobotSoccer

This repository contains code that controls a Roomba with a Kinect camera to play soccer.
I've also included code that was made in the process to test out certain features.
This was a class project.
The code was written in Python and the communcations to the robots were handled with ROS.

The "code" folder is where all of the actual python scripts for manipulating the robots. It should contain:
  PrisonBreak.py
, BallFinder.py
, RobotPuppy.py
, Dodgeball.py
, RobotSoccer.py

RobotSoccer.py and BallFinder.py are both the finished product. They both need to be ran in order for the Roomba to play soccer.

1. PrisonBreak.py is the first code that I wrote. It basically has the Roomba moving straight until it hits an object. Then its bump sensors will tell the robot to back up then go forward and turn. This was name PrisonBreak because eventually it will bump and go forward until it finds the exit of the room.
2. BallFinder.py is used to find the ball with its camera. The ball was yellow, so this code checks an array of pixels in its camera feed to see which shade of yellow fits with the ball. This probably would've been better to use edge dectection or shape dectection, but finding color was the quick and dirty way.
3. RobotPuppy.py is used to have the Roomba come up to the ball and kick it. RobotPuppy.py would initially go into search mode (it spins in circles) until it finds the ball using BallFinder.py. Then the robot approaches and kicks.
4. Dodgeball.py is used to go around the ball and then kick it. This also goes into search mode and does everything RobotPuppy.py does except for kicking it. Instead, it goes around the ball then kicks it.
5. RobotSoccer.py combines elements from PrisonBreak.py and Dodgeball.py. RobotSoccer.py finds the goal, which is an AR tag. Then it goes into search mode(spins in circles) until BallFinder.py finds the ball. RobotSoccer.py then approaches, lines up, finds the shortest path around the ball, goes to that point, and lines up again and kicks the ball towards the AR tag.

In the end, I played soccer with other classmates and ended up in the finals. Sadly I didn't win. I learned a lot though!
Also, I noticed that the comments in the code get sparser as development continued (PrisonBreak.py to RobotSoccer.py). This is a mistake on my part because I didn't think I'll be uploading it to GitHub. But if you read PrisonBreak.py to RobotSoccer.py you can see a lot of ideas are repeated, so code in RobotSoccer.py could've been already commented in RobotPuppy.py. Anyways, I know it should've been commented better. If you have any questions I'll be glad to answer them to make up for this mistake!

catkin_ws.zip was the actual ROS enviroment I worked in. The actual code that does everything important is in the "code" folder!
