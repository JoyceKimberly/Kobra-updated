# Kobra-updated

I've been bringing the firmware for the Kobra up to date with Marlin as much as possible, however this has now reached the point where people with more knowledge on the subject need to go over it.
It compiles and runs on my Kobra successfully, but I lack the experience to make the final changes to HAL and take any other changes out of Marlin and into the lib folder so new Marlin changes can simply be dropped in.

It compiles according to the instructions given here 
https://www.reddit.com/r/anycubic/comments/y2waxu/tutorial_how_to_build_anycubic_marlin_source_code/

I've set the F_CPU value in uVision after making the firmware print out the value together with the other stored values and copying it from Pronterface, I have no idea if this value is the same for all Kobra's, so if it doesn't work, just remove the value from uVision, compile and flash, and read out the value when sending M503.

With many thanks to
https://github.com/jojos38/anycubic-kobra-improved-firmware
https://github.com/shadow578/Marlin-H32

# Warning

Please do not install this on your machine in order to use it. I cannot promise it is safe, it really needs some proper programmers to go over it first.

# To do

- Fix menu getting stuck after auto-leveling
- Finish HAL
- Make MarlinSerial work
- Find out what the change to AWAIT_TIMED_PULSE means and how to apply it outside of stepper.cpp
- Completely redo kobra extui and dgus so it makes sense
- Find a way to get rid of having to use uVision to compile