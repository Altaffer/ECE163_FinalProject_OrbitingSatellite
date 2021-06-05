Richard Owens (rivowens@ucsc.edu)
Jason May (jwmay@ucsc.edu)
Luca Altaffer (taltaffe@ucsc.edu)
Mana Iwata (miwata@ucsc.edu)

There are 3 scripts to run:
(primary)
Python SimulateVGM.py
python SimulateVCLC.py
(secondary)
python testCLC_orientation.py
python testCLC_position.py

Simulate VGM and VCLC - these simulators run two of our modules, Vehicle Gravitational Model which models the forces and control forces on the spacecraft and Vehicle Closed Loop Control which applies VGM and our controls system to control the spacecraft. VGM is currently working properly and the instructions for how to set up test scripts are in the file, with a test script argument class that has labeled arguments and a runTest function that returns an assortment of state plots. One test has been set up currently so as to not spam graphs to the user, if you wish to test different features, or isolate features, of VGM look for the test argument class with clearly labeled inputs.

VCLC is set up in a very similar way. There is an argument class and the runTest function. VCLC is still a work in progress as our entire simulator is not up and running, though it still functions properly with issues such as improperly tuned gains. VCLC also offers plots in both the normal NED frame and our orbital frame TOR, which is the vector tangent to the orbit, orthogonal to the orbit, and a radial vector towards the center of the orbit. Again, one test has been set up and looks to the arguments class for instructions on testing different features of the simulator.

The testCLC_orientation script demonstrates the satellite aligning with an arbitrary commanded roll, pitch, and yaw. Figure 1 displays the angular misalignment about each axis. Figure 2 displays the rate at which the angular misalignment about each axis is changing. Figure 3 displays the reactor wheel commands. Figure 4 shows the satellite’s yaw, pitch, and roll as it rotates towards alignment.

The testCLC_position script demonstrates the satellite controlling it’s orbital velocity, orbital radius, and distance from the orbital plane. This script makes the assumption that the satellite is relatively close to the trim state, such that the orbital velocity, orbital radius, and distance from the orbital plane are independent from each other. Figure 1 “orbital positon po” shows the offset of the satellite from the orbital plane. “Orbital position pr” shows the radial distance from the center of the earth. “Orbital position pt” an be ignored, as the orbital frame would normally follow the tangential position. Figure 2 shows the velocities along each of the orbital axises. Figure 3 shows the thruster actuation.

The base of our project consists of our dynamics model, gravitational model, disturbance model, and closed loop control. These modules have been completed with extensive debugging and testing. With this complete we are ready to move on to creating additions to the simulator by modeling more complex or realistic components. Ideas for this would be to model our satellite coming into orbit from an Earth based launch, modeling the orbits of the celestial bodies, or even including magnetorquers into the attitude control.
