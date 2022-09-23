# Warm-up Project
**A Computational Introduction to Robotics**

Liv Dawes and Allison Li

## Overview
The goal of this warmup project is to gain familiarity with both ROS2 and the Neato. We used ROS2 to implement 6 behaviors on our Neato: teleop, driving in a square, wall following, person following, obstacle avoidance, and a finite state machine. We were able to test and debug our code using a physical Neato as well as a simulation. 

## Teleop
### Description
Our goal with teleop was to be able to remotely control the movement of the Neato through the computer keyboard. 

https://user-images.githubusercontent.com/86380596/191907234-733bdc6d-feec-4b0f-b658-50dbd7c995b6.mp4

### Strategy
We chose to use the WASD keys and SPACE on the computer as those are movement keys that many people are already familiar with. The W key moves the robot forward, A and D turn, and S moves the robot backwards. Pressing SPACE stops the robot. Unfortunately, this means that the robot is only able to move or turn, it is not able to turn while driving. We chose to implement it this way for the sake of simplicity, but if we had more time, it would be good to implement more complex controls that allowed the robot to both turn and move at once. 
### Code Structure
The teleop.py file contains a TeleopNode class which houses the logic for the teleop behavior. TeleopNode has one publisher that publishes a Twist message to the /cmd_vel topic to control the movement of the robot. It also contains a function getKey() which returns the key being pressed on the keyboard. 

## Drive Square
### Description
Our goal was to drive the robot in a 1 meter square. We tried to be as precise as possible with our distances and angles to make it as repeatable as possible. 

https://user-images.githubusercontent.com/86380596/191907775-92bd7fa7-cf82-4eba-b948-331ded9d0a6e.mp4

### Strategy
For this behavior, we used a timer and then had the robot alternate between driving or turning for a set amount of time. The velocity input into the robot is already in meters per second, so it is not hard to calculate how long a robot must drive at any given velocity to travel 1 meter. In our case, our robot drove at .3 m/s so it drove for approximately 3.3 seconds. The angular velocity is in radians per second, so it is also easy to calculate the appropriate amount of time based on a given angular velocity. We used an angular velocity of 0.6 with a time period of 3 seconds. One limitation of this strategy is that it relies on a timer without any sensor feedback. This means if the robot's wheels slip on loose terrain causing it to spin in place, the robot has no way of correcting for that. Additionally, there is no way to change the speed without manually recalculating the correct time intervals. A future improvement would be to make the speed and time variable so that you could change one and automatically calculate the other. 
### Code Structure
The drive_square.py file contains a DriveSquareNode class which houses the logic for driving in a square. There is one publisher that publishes a Twist message to the /cmd_vel topic to control the movement of the robot. We also have a timer with a series of if statements that change the behavior from driving in a straight line for a set amount of time to turning for a set amount of time. It is essentially a finite state machine but it uses if statements and a time to switch between states. 

## Wall Following
### Description
The purpose of this behavior is to have the Neato drive in a straight line while maintaining a constant distance from a wall. 

https://user-images.githubusercontent.com/86380596/191907693-d2825eb1-d5dd-496a-a0e8-35d0eb69be6c.mp4

### Strategy 
First, we used the built in lidar sensor to find the distance from the right and left side of the robot to the wall. Then, we compared the distances to find the closer wall. This is the wall that we will follow. Then, based on which wall we were following, we found the distance to the wall 45 degrees ahead and behind the robot. If the robot is parallel to the wall, these distances will be equal. If the distances are not equal, the robot is not parallel and can self correct using the difference between the two distances. The robot drives at a constant speed while correcting its angle to stay parallel to the wall. 

<img width="362" alt="picture 1" src="https://user-images.githubusercontent.com/86380596/191909027-db3600ad-0b0f-42cc-8ca1-ecba62295935.PNG">

Figure 1. Measurements from the Neato LiDAR sensor used for wall following. For each measurement, a 20 degree range (± 10 degrees) of data points was recorded for redundancy, filtered to remove dropped scans (0s), and averaged.

<img width="393" alt="picture 2" src="https://user-images.githubusercontent.com/86380596/191909230-9f08ae24-f7a5-459e-bc5b-f47200d818e8.PNG">


Figure 2. Wall following logic diagram. The robot chooses which wall to follow based on whether a directly left or right scan is closest, then compares the front and back measurements to determine which direction to turn to orient itself parallel to the wall.
### Code Structure
The wall_follower.py file contains a WallFollowerNode class which houses the logic for the wall following behavior. The WallFollowerNode has a publisher that publishes to the /cmd_vel topic to control the movement of the robot and a subscriber that subscribes to the /scan topic to retrieve the distance data from the lidar sensor. We also incorporated a parameter to allow the user to change the value of the proportional controller from the command line. This value affects the speed at which the robot turns to self-correct when it is not parallel to the wall. Finally, we also incorporated a marker to visualize the position of the wall in rvis. 

## Person Following
### Description
For person following, our robot identified a “person” in an empty space and then drove towards the person.

https://user-images.githubusercontent.com/86380596/191907636-2dbf37b8-77f4-442b-9a43-55b07adfc4be.mp4

### Strategy
For this behavior, we assume that the robot is an empty space aside from the target. That means there are no nearby walls or obstacles. First, we take the entire LIDAR scan and filter it to remove all points outside a given radius. We used a radius of .75 meters because it seemed like a good balance. Then, we found the mean of the points present in the filtered data. This assumes that all points come from the same target, so it is not robust to multiple targets. After finding the mean of the data, or the centroid of the target, we calculated the angle between the target and the robots current heading. Then, we turned the robot toward the heading while driving at a constant velocity. One primary limitation of this strategy is that the robot can not handle multiple targets. If there are multiple targets, it will still try to find the mean of the targets and end up with a point in between. 

<img width="269" alt="picture 3" src="https://user-images.githubusercontent.com/86380596/191909569-46025231-ca59-44f4-ba12-bad20dc64449.PNG">

Figure 3. Person following logic diagram. The robot only considers points within a 0.75m radius. It then finds the mean point, or centroid, of all points within range, determines the angle between the robot’s current heading and the centroid, and turns to follow accordingly.
### Code Structure
The person_follower.py file contains a PersonFollowerNode class which houses the logic for the person following behavior. The PersonFollowerNode has a publisher that publishes to the /cmd_vel topic to control the movement of the robot and a subscriber that subscribes to the /scan topic to retrieve the distance data from the lidar sensor. We also incorporated a parameter to allow the user to change the value of the proportional controller from the command line. This value affects the speed at which the robot turns towards the person. Finally, we also incorporated a marker to visualize the position of the person in rvis. This marker was especially helpful at visualizing when the robot picked up multiple targets.


## Obstacle Avoidance
### Description
Our goal was to have the robot drive in a straight line while avoiding obstacles placed in its way. 

https://user-images.githubusercontent.com/86380596/191907565-7591b894-b06c-45f6-bd63-cb8e4b54d75e.mp4

### Strategy
For this behavior, we restricted the LIDAR scan to just 180 degrees at the front of the robot. This means it is unable to see behind itself.  We want the robot to drive in a straight line but also avoid any obstacles it sees. To do this we created a curve representing the desired heading. For our purposes, we created a normal curve spanning -90 to 90 degrees with a peak at 0 degrees (representing straight ahead). The height of each point corresponded to how desired that heading was: straight ahead was most desired and either side was least desired.  Then, we processed the LIDAR data in a similar way. We created a vector with all the points where values farther away were higher and therefore more desired and values closer were lower and therefore less desired. Then we used elementwise addition to add the two arrays and come up with a final array representing the desirability of each direction. At this point, we used a moving average to smooth out the curve and found the maximum and turned the robot towards that point while moving. 

<img width="468" alt="picture 4" src="https://user-images.githubusercontent.com/86380596/191909804-8a3e85d0-edf2-4f50-a3cf-d3f0ed809979.PNG">

Figure 4. Obstacle avoiding example situation and logic. Since large scan values indicate objects further away, a 180° LiDAR scan range (-90° and 90° from the forward heading of the robot) is added to a normal curve and the maximum is taken as the desired angle that the robot should turn towards. Using the normal curve causes the robot to favor moving directly forwards when scans are large in multiple directions. To center the robot within gaps between obstacles, a moving average filter is applied over the scan data to smooth sharp edges.
### Code Structure
The obstacle_avoider.py file contains a ObstacleAvoiderNode class which houses the logic for the obstacle avoidance behavior. The ObstacleAvoiderNode has a publisher that publishes to the /cmd_vel topic to control the movement of the robot and a subscriber that subscribes to the /scan topic to retrieve the distance data from the lidar sensor. We also incorporated a parameter to allow the user to change the value of the proportional controller from the command line. This value affects the speed at which the robot turns towards the person. Finally, we also incorporated a marker to visualize the position of the person in rvis. We decided not to implement a marker with this behavior. 


## Finite-State Control
### Description
Our finite state machine switched between two behaviors, our person-following behavior and a new spin dance behavior. 

https://user-images.githubusercontent.com/86380596/191907441-56763522-a6b9-4c72-9142-1ffe2c6174dd.mp4

### Strategy
The robot begins in the default person following behavior where it locates a person and drives towards them. When the robot reaches the person and triggers its bump sensor, it switches to its second state. In the second state, the robot reverses slightly to clear the person and then spins rapidly. The robot remains in this state until its bump sensor is triggered again and then it switches back to its first state where it person-follows. 

<img width="428" alt="picture 5" src="https://user-images.githubusercontent.com/86380596/191910014-582a4d81-6842-49a5-9a48-b34ad9b7f645.PNG">

Figure 5. Finite state diagram for a Neato controller. The Neato will default to person following behavior, but when a bump sensor is activated it will reverse and spin around until a bump sensor is activated again, at which point it will return to person following behavior.
### Code Structure
The finite_state_controller.py file contains both a State class which in an enum that contains the names of states and a StateMachineNode class which houses the logic for the finite state machine. The code switches the same person-following code as earlier and code commanding the robot to reverse slightly and spin.

## Conclusion
### Challenges
As we completed this project we encountered several challenges. THe largest challenge we encountered was dropped points and out of range points. In the virtual environment, the LIDAR received perfect data and never dropped a point. However, in real life, the LIDAR was not perfect and occasionally dropped points. In both the virtual environment and real life, points were often out of range. In the simulation, this results in the points being infinity. In real life, the points were zero when they were out of range. This means we had to do a lot of filtering on our LIDAR data to account for all the nuances and make sure we interpreted the data correctly. 
### Improvements 
Some improvements were mentioned earlier where relevant for specific behaviors. Overall, if we had more time, we would make our code more robust. One weakness is that our code does not effectively deal with things like multiple targets for person following or corners in wall following. It would be good to learn RANSAC and implement it to parse the LIDAR scans and determine chapes in the environment. This would allow us to respond differently to different features. 
### Take-aways
One major take-away from this project was how many debugging tools there are. We used a lot of print based debugging because thats a strategy that we’re both comfortable with. But we also started to use bag files as well as visualizing data with markers in rvis. Using rvis to visualize things was especially useful because it gave a better intuitive understanding of what our code was doing and what issues were coming up. Another useful tool was debugging both in simulator and real-life. Both of the interfaces come with their own issues and quirks. Testing in both helped us write more robust code. 

