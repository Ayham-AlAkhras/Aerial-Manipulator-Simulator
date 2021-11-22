# Aerial-Manipulator-Simulator

The simulation was initially created in MATLAB simulating an Arduino microcontroller’s
output commands. First, all system parameters were defined, including link lengths, masses,
angle limitations, and desired final position. The system then enters a loop, wherein in
computes inverse kinematics utilizing the symbolic toolbox in MATLAB. This toolbox
numerically solves systems of equations and retrieves accurate outputs.
Due to angle identities, the inverse kinematics produces two possible solutions. One
named the nominal position and the other named the inverted position. Of these two positions,
the one that optimally fits into the drone’s workspace and prevents interference and large angle
changes is selected using an if statement.

Once computed, the program calculates the difference between initial and desired angles. This
difference is utilized within the loop to scale down the actuation movement such that both links
arrive at their final positions at the same time. This provides a rudimentary path planning
function built into the inverse kinematic system. Results show that the path planning implemented improves performance allowing both links to
move smoothly and simplifying the overall motion of the manipulator.

Once the inverse kinematic final angle positions were found, an algorithm initiated to simulate
and represent the movement of the links. The code begins at a predefined starting point and plots
a graphical representation of the manipulator’s planar motion as it increments by 1 degree or less
to the final position. The 1-degree motion is derived from a 1:1 gear ratio between the servo
motors and links (this changes when implementing the actual design in Arduino) During this
incrementation, three main algorithms are run:

-Forawrd kinematics: solves the positions for graphical representation.

-Center of gravity calculation: computes the center of graavity of the system and conputes the counter-balance shift required for compensation.

-Stopping conditions: stops the algorithm when needed.

After the stopping conditions are met, all loops are exited and the collected data is graphed intuitively.
In the main graph, the lines represent the links, whereas the points represent battery displacement, COG,
and payload location. The red color indicates the starting position, blue indicates intermediary
positions, and the green position is the final position of a pre-determined arbitrary motion.
