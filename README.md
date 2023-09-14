# CompRoboWarmup2023


    For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

TELEOP
    The teleop node is meant to control the robot forward, backwards, left, right, as well as left-forward, right-forward, left-backward, and right-backward. 

    
    For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.
    How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.

WALL FOLLOWER
    The wall follower node allows the robot to sense the wall nearest to its right, and drive parallel to it. 

    The node subscribes to the /scan topic to access the robot laser scan, and publishes to /cmd_vel. 
    # use rqt to reference the subnode diagrams 

    The bot uses its laser scanner at both 45 and 135 degrees (relative to the robot's coordinate frame) and finds how far away it is from the wall. After calculating the difference between the angle of the wall and its own angle relative to the robot's frame, it rotates itself by the value to face a parallel path to the wall and rolls ahead indefinitely. 

    The primary limitation of this node is that it can only sense a wall to its left, and only two angles are used to sense the wall for the sake of the math used to determine the angle the robot must rotate. One of the angles is further behind the robot than the first angle, and should the wall be too short, behind the robot, it will fail to sense it correctly and make the calculations wrong. 
    
PERSON FOLLOWER 
    The person follower node allows the robot to sense the closest "person" within 3m of it, then turn/drive towards it. 

    The node subscribes to the /scan topic to access the robot laser scan, and publishes to /cmd_vel to command the robot how to move.

    The bot iterates through each angle of the lidar data, and saves only the angle of which the closest distance was scanned. Using this, the robot will gauge which side of the robot this closest object was, then begin to turn to face that direction. 

    The primary limitation of this node is that it cannot determine what is an object versus a person.
