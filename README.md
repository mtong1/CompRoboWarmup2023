# CompRoboWarmup2023

##### By An and Madie

### TELEOP

The teleop node controls the robot’s forward, backwards, left, and right movement. We wanted to use the W, A, S, and D keys or WASD keys on the computer keyboard as arrow keys since they are popular for gaming. We used W and S to control the linear x movement of the robot and A and S to control the angular velocity.

Our strategy at a high level was understanding the robots coordinate system. Once we had a good understanding of how it moved, we could send velocity commands using twist and publisher. We wanted clean movements so we set linear velocity to zero when it was turning and the angular velocity to zero when it was moving forward and backward.

![](images/image6.png)

### DRIVE SQUARE

![](images/drive_square.gif)
The drive square node commands the robot to draw a one meter square. The movement was hard coded based on speed and time. The robot moves forward 1 meter at a set speed for a set time then turns 90 degrees four times within a for loop. The time was calculated by dividing the distance or angle by the current speed.

$ Distance / linear speed = time \
$ Angle / angular speed = time

A limitation of the time calculations is it assumes the robot goes from 0 to the set velocity instantaneously as opposed to accelerating up to the speed, thus the time calculation is slightly lower than the true time it takes to turn or move 1 meter. The time difference was noticeable in the 90 degrees turns, so we added an acceleration time that adds more time to allow the robot to complete the full turn. The acceleration time was calculated through trial and error.

![](images/image4.png)

### WALL FOLLOWER

The wall follower node allows the robot to sense the wall nearest to its left, and drive parallel to it.The node subscribes to the /scan topic to access the robot laser scan, and publishes to /cmd_vel.

![](images/image.png)

Our strategy was to have the robot look at the laser scanner message at both 45 and 135 degrees (relative to the robot's coordinate frame) in order to find how far away it is from the wall. Using trigonometry, we calculated the (x,y) coordinates of the wall vector relative to the robot, and shift the vector over the robot plane. We simplified the math by hand and determined a single equation (shown below) we can use to directly input the distances sensed from the robot in our code. After calculating the difference between the angle of the wall and its own angle relative to the robot's frame, it rotates itself by the value to face a parallel path to the wall and rolls ahead indefinitely. The math is shown in the image below.
![](images/photo5.jpg)

![](images/wall_follower_math.jpg)

The trickiest part of this implementation was trying to get our math to work. The robot has to calculate the vector of the wall in terms of coordinates, which requires trigonometry. We had to work out the math on paper before even attempting to code it, and even with the code we ran into many technical errors that combined logistic math mistakes and coding mistakes, which made it more difficult to debug. Though originally the wall follower code had been working great, we found later on in the project that it was no longer turning the correct angle. We believe that though our understanding of the math required to implement this node is correct, our understanding of the various coordinate frames at play is somewhat flawed, which is leading to the implementation of the math being incorrect.

The primary limitation of this node is that it can only sense a wall to its left, and only two angles are used to sense the wall for the sake of the math used to determine the angle the robot must rotate. One of the angles is further behind the robot than the first angle, and should the wall be too short, behind the robot, it will fail to sense it correctly and make the calculations wrong.

![](images/image1.png)

![](images/rviz_wall_follower.gif)

### PERSON FOLLOWER

![](images/person_follower.gif)
The person follower node allows the robot to sense the closest "person" within 3m of it, then turn/drive towards it. The node subscribes to the /scan topic to access the robot laser scan, and publishes to /cmd_vel to command the robot how to move.

The bot iterates through each angle of the lidar data, and saves only the angle of which the closest distance was scanned. Using this, the robot will gauge which side of the robot this closest object was, then begin to turn to face that direction. Once it senses that this closest object is in front of it, it will start to drive forwards.

The trickiest part of implementing this task was understanding how ROS timers and loops worked first, which helped to simplify the problem. When implementing this, we wanted to avoid calculating a “turn time” for the robot before commanding it to move. We realized after discussing with CA’s that a quick and easy solution for this problem was just a built-in part of ROS; if we just commanded the robot to turn endlessly, the object would eventually be in front of the robot, in which case the loop would reset and the robot would realize it could start going straight.

The primary limitation of this node is that it cannot determine what is an object versus a person. It will simply follow any object that it senses with its laser scan. Additionally, it will only follow a “person” that is within 3 meters of it, rather than the “closest person” in general. This limitation was intentionally added to simplify the task; Limiting the robot’s “following” range helps it to avoid following unwanted objects such as walls or further objects.

![](images/image2.png)

### OBSTACLE AVOIDANCE

![](images/obstacle_avoider.gif)
We implemented the obstacle avoidance node for the robot to avoid any object that is within 5 meters of its front side. The node subscribes to the /scan topic to access the robot laser scan, and publishes to /cmd_vel to command the robot how to move.

The bot was implemented similarly to person follower, but with a reversed behavior. The bot would sense if there was an object within 5 meters of the front side, specifically a 40 degree range about 0 degrees. Should it sense an object, it would then determine if the object was more towards its left or right (this can be determined given the angle saved is from 0-20 degrees, meaning the object is on its left, or 340-361 degrees, meaning the object is on its right). Depending on which side the object was more towards, the robot would turn towards its opposite direction until the object was out of its front sight. Only then would it continue driving forward.

The trickiest part of implementing this was actually trying to determine the best way to implement this task. Because of the possible complexity this problem could have, we had to decide what is the best way to simplify this problem to a more solvable manner, and how much abstraction is still enough for the code to still technically do the task it was asked.

![](images/image3.png)

### FINITE STATE CONTROLLER

![](images/finite_state_controller.gif)
The behavior of our finite state controller combined person follower and obstacle avoidance. The states were “predator” where it tried to reach the closest object and “prey” where it would try to run from the other objects. We combined the code by creating a state variable, and nesting each state’s code within an if statement that checks the state. The if statement is within the run loop and within the parse_scans methods since the different states used different techniques to read the lidar scan. We detected the transition between states when the robot is less than 0.3 meters away from its object. The initial inspiration behind the states was tag.

![](images/image5.png)
![](images/image7.png)

### CODE STRUCTURE

Our code is placed in a ROS package called warmup_project. For each assignment, we created a different node. Structurally, each node required some way to communicate with the robot about what we want it to see or do. Thus we included objects for publishers and subscribers to the topics /cmd_vel and /scan to tell the robot how to move and to scan, respectively.

For each node, we separated our code into methods according to what parts worked together. For example, we had methods called run_robot() that when called, would publish messages to /cmd_vel about how to move the robot according to calculations run in the run_loop() method, or information processed in the parse_scan() method.

### CHALLENGES

We faced multiple debugging challenges with the robot not interacting as we expected. Some common problems were forgetting to publish and not explicitly setting a robot’s speed at every set. We spent the most time debugging the math and implementation of wall follower. More detailed challenges were mentioned above.

### IMPROVEMENTS

We first wanted to complete the minimal viable product for each section before improving any one section. We would use odometry for the move square node as opposed to hard coding the movement. We ran into issues with the turn time calculation which could be solved using the robot’s odometry.

For the wall follower, we would improve the implementation to detect if the wall is on the left and the right. Our code currently only works if the wall is to the left of the robot.
If we were to try to improve the obstacle avoidance, we would probably implement this similar to the gauntlet challenge in QEA3, by mapping objects surrounding the robot and giving each object a “weight” of importance dependent on its size and proximity. This would allow the robot an easier time of prioritizing and determining the most optimal path.

### TAKEAWAYS

An important first step that should not be skipped is understanding the math and expected behavior of the robot before coding. Debugging was more efficient and successful when we could clearly see the differences between our expectations and the robot. In addition, print statements were extremely helpful for debugging. The project gave us a good foundation for exploring ros and various topics where we could communicate and control a neato.
