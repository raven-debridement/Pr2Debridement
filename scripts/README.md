This folder contains the files that makeup the main pipeline for Pr2Debridement

Outline of the file purposes:

Master.py
 - Has the master pipeline and main loop

ArmControl.py
 - Controls the arms of the pr2 using either
   planning with trajopt, no planning, or servoing

CommandGripper.py
 - For opening and closing the gripper

ImageDetection.py
 - For visually detecting the object, gripper, and receptacle

Util.py
 - Random useful functions

Constants.py
 - Prevalent constants