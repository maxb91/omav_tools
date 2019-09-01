# omav_tools
Tools for overactuated vehicles.

Includes a simple trajectory generator that reads in text files to generate 6-DOF trajectories.

# Trajectory generator
The trajectory generator reads in a waypoint list text file, plans a path, and publishes it on request (just one single message).
The text file needs to be structured in following format:

x y z yaw pitch roll

Each line represents a waypoint. Yaw pitch roll are Euler angles, rotating from the world to the body frame (yaw-pitch-roll).
