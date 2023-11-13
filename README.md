# Spotter
Spotter project for ELEC 390 course

This project consists of a fitness assistance that tracks the angle of the knee and back of a person during a workout to enhance their
posture and form for better training. The knee motion is tracked using two IMU sensors (LSM9DS1) and the bacl angle is measured using a
flex sensor. Both of those are connected to an arduino D1 R1 via wires which then, for its part, transmits the data received to an online
database (using Firebase) via wifi. This data is all displayed on a mobile application where resutls can be visualised on a graph and
recommendations are given to improve form. It also controls the arduino board to start the data sampling and to calibrate the devices for
optimal accuracy. This project is only a prototype that was designed and developped in about 3 months.

More details can be found in the Final_Presentation_-_Spotter.pdf and a demontration/mock advertisement ofd the product can also be seen in Spotter_Advertisment_V2.mp4.

- final_code.ino is the code running on the arduino board to sample and transmit the data from the sensors to the database.
- simulation.ino is the simulation code that was used to determine the best algorithm for sampling angle data from the sensors to achieve best accuracy.
- Controller, Model and View folders contain the java files behind the mobile application
- drawable folder has images and xml code for the visuals of the application
- layout folder contains the xml files to define the placement of elements on the interface (UI)
- values folder contains variables used as substitutions in other files for colors, strings, themes...
