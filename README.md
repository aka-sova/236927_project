# 236927_project
Final project - RC obstacle avoidance


Added the calibration algorithm
in order to interpolate the exact encoder commands

For long distances, using the 'Drive' command should be coupled with the constant feedback from the system on the location
of the agent. Several approaches can be used:

1. Give a continuous command to an agent to move to a target. 
   As speed of agent increases, the 'brakes' period from the moment of the last command till the complete stop increases.
   So calculate this brake path / speed relation, so that the agent will stop in time.
   Or calculate experimentally the time when the robot has to stop.
   Calculate its velocity???




NEXT:

1. Finish the GOTO algorithm using those simple commands (turn, drive_distance)
2. See how we can implement the drive command for long distances. Drive 


3. Fix for when receiving -9999 -9990 for position!!!!!!  causes problems
