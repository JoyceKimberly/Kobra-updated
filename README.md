# Kobra-updated

I've been bringing the firmware for the Kobra up to date with Marlin as much as possible, however this has now reached the point where people with more knowledge on the subject need to go over it.
It compiles and runs on my Kobra successfully, but I lack the experience to make the final changes to HAL and take any other changes out of Marlin and into the lib folder so new Marlin changes can simply be dropped in.

It compiles according to the instructions given here 
https://www.reddit.com/r/anycubic/comments/y2waxu/tutorial_how_to_build_anycubic_marlin_source_code/

I've set the F_CPU value in uVision after making the firmware print out the value together with the other stored values and copying it from Pronterface, I have no idea if this value is the same for all Kobra's, so if it doesn't work, just remove the value from uVision, compile and flash, and read out the value when sending M503.

Please note that any tuned values (like PiD) in configuration will be set to my printer's as it's a pain having to redo them when you're flashing new firmware all the time.
It's for that same reason that I will not be releasing any pre-compiled bin files.

The configuration is set to use the Vyper lcd ui on purpose as it is almost identical to Kobra's.

The following sources have been invaluable 
https://github.com/jojos38/anycubic-kobra-improved-firmware
https://github.com/shadow578/Marlin-H32
https://github.com/malebuffy/Kobra-Max-Cobra-Kai-UI-and-Firmware

# Warning

Unfortunately I cannot give any promises or guarantees, I just have my one printer to try things on, so I cannot test for any other circumstances or variables. Which means that you are fully responsible for what happens when using this firmware.

# To do

- Fix menu getting stuck after auto-leveling. For now just press the middle of the screen when it's done.
- Finish HAL
- Make MarlinSerial work
- Completely redo kobra extui and dgus so it makes sense
- Find a way to get rid of having to use uVision to compile
- Update SD. This is the only folder that's relatively unchanged, as I figured that as long as it works, doesn't matter much :P