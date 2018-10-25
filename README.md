# Monash FYP Air Hockey Table
Code and documentation related to the Monash FYP Air Hockey Table

# New Projecteers
Firstly, good luck guys. This project is different from a lot of the others on offer.
Instead of designing and building out project, we spent most of our time trying to decipher the mess of wires and connnectors
which are attached. It is achievable though, so stay with it.

Make sure you get acquainted with Ian Reynolds as soon as you can. He is the original engineer who designed the circuitry and mechanics.
His knowledge is going to be invaluable to you over the coming year, especially if he has rewired the project recently.

Make this project your own. We made the mistake of thinking the whole point was to make the table play air hockey, but it's not.
The point is for you to design the control software for specific components, and then demonstrate them using the table.
Whether or not the system works for the final demonstration is going to depend entirely on how well you've designed the software.

# Suggestions, tips and things we discovered

* The table surface is warped, so level it off as best you can. It won't ever be perfect, just do the best you can.
* The puck detection has no invariance to light. One bright light above the table is enough to ruin everything, the same as one shadow.
We weren't sure how to make the vision less dependent on lighting while still using the FPGA for decoding. People suggested using a
Raspberry Pi and OpenCV, but then the whole system is changed. You could probably talk to Tom Drummond, if you can convince him to make time.
* There is tons of signal noise on the encoders, limit switches and motor signals. If someone hasn't done it yet, you should probably use
shielded wires and re-route the paths. Keep the AC wires away from the DC wires.
* The motion profile generator we created could use some work. It's accurate to with 100 encoder counts, less than a millimetre.
It also won't start building a new profile until it gets given a new target. It would be better to instead use some sort of deadband,
but lead to oscillations for us.
* You could also use a PI controller instead of just a proportional controller, but won't be worth it until the accuracy in puck tracking is
improved. The steady-state error is small enough not to cause problems yet.
* The encoders are wired to use a single ended signal. There are wires on there for differential signal, but they have just been left
floating. Making them differential could help limit noise in the system.
* The camera feed is 728x488 pixels, but limited to 640x480 in Verilog. The optic centre isn't in the middle of the image, but it is
recorded correctly in the code. We moved the distortion correction to uC RTOS from Verilog, because it's more accurate. You can check
the attempt from 2017 to see how to put it back into Verilog, but use the parameters we found.
* The decision timer for choosing when to begin striking the puck is very poor. The resolution is too large of a stepsize. You should
probably figure out a better way to do this.
* The final demonstration isn't very interactive. Try putting physical dials or sliders on the table to control things (eg. motor speed,
playing style). These might win you a prize at SparkNight.
