System Technical Documentation:
Various connections will have to be made between the microcontroller and the rest of the sorting system. Circuit diagrams, block diagrams, and hardware and software system algorithms detail the logic and processes used that drive and integrate the system.
In order to control the stepper motor, PORTA is configured as an output part and is connected to the various inputs of the K CMD - L298 Integrated DC Motor Driver (stepper motor). PORTB is also configured as an output and is connected to the various inputs of the Pololu MD01A - VNH2SP30 Integrate DC Motor Driver (DC Motor). PORTC is configured as an output port and is connected to the various input pins of the LCD display. PORTD is configured as an input port and is connected to the various sensors mounted on the sorting system, such as the Hall Effect Sensor (HE) to PD3, and the two Optical Sensors,OR to PD2 and EX to PD0. These inputs trigger interrupts INT0-3. PORTE is also configured as an input port, and is connected to the two buttons responsible for pause/resume and ramp down functions, PE4 and PE5, and trigger interrupts INT4-5 respectively. PORTF is an input port connected to the Reflective Sensor (RL), and is the ADC conversion port, PF1.

System Algorithm:
When the system starts up, the home position of the sorting tray must be located to provide a reference position for the system. The home position is found by continuously rotating the sorting tray using the stepper motor until a low signal from the Hall Effect (HE) signal is detected. The home position will be identified as the black plastic bin. A flowchart that illustrates how the home position of the sorting tray is located is provided below in Figure 3. Proceeding a successful homing of the stepper motor the DC motor will start up. In order to successfully sort all of the cylindrical objects, the following chart algorithm depicted in Figure 4 is utilized. When an object crosses optical sensor 1 (OR), the corresponding ISR will call for the start of ADC conversions to find the optimal ADC value from the reflective sensor. A flowchart illustrating how the optimal ADC measurement is determined is provided in Figure 5. The optimal value is then compared to preset values for each item type (these values are determined during the calibration process detailed in Section 4.1), then the object is assigned into a category and placed into the object array. A flowchart illustrating how the object type is determined is provided in Figure 6. When an object crosses optical sensor 2 (EX), the DC motor will stop and the stepper motor will rotate to the desired position. A flowchart illustrating how the sorting tray is correctly positioned is provided in Figure 7. Finally, The DC motor will start up again, and the object will be placed into the sorting tray. 

System Flowcharts:
In the case of an external interrupt such as the pause/resume button, the belt will come to an immediate stop and the amounts of each item in the bucket, as well as the total amount of items being processed on the conveyor belt will be displayed on the LCD. Pressing the button again will resume the system to its regular operation. In the case of a ramp down, the system will start a timer to allow all current pieces on the conveyor belt to be deposited into their respective buckets, then the system will come to a complete halt, similar to a kill switch. 
Since the determination of object type relies on the ADC measurement from the reflective sensor, it is crucial that the optimal ADC measurement is determined. The software approach to obtaining the optimal ADC measurement that was utilized is as follows: As soon as an object crosses the optical sensor 1 (OR), the corresponding ADC ISR will start a single ADC conversion. Once the conversion is finished, the ADC value will be compared to the current optimal value. The current optimal value is a temporary dummy variable that will store the most reflective (lowest) ADC measurement. If the ADC measurement is lower (more reflective) than the current optimal value, the ADC measurement will be passed to the dummy variable. If the object is still in front of the optical sensor, a new ADC conversion will begin and the optimization process will repeat. Sequentially, the optimal ADC measurement will be compared to the ranges defined for each item, and the item will be assigned a category accordingly. 
The stepper will turn once an item has reached the final optical sensor on the conveyor belt. Depending on the item’s category and the current position of the bucket, the stepper motor will turn either 90 degrees clockwise, 180 degrees clockwise or 90 degrees counterclockwise. In the case the stepper motor is already in the proper position the motor will not turn.

Maxium Operating Parameters:
PWM for DC Motor: 85%
The system demonstrated that at this PWM setting, the motor operates at its highest speed. However, it's noted that at this maximum speed setting, objects sometimes did not stop at the end of the belt and fell off (due to inertia), indicating a potential for high error rates or unsatisfactory sorting performance index (SPI). Furthermore, higher speeds may affect the user’s ability to place items on the conveyor belt running at full speed without pieces falling over.
Stepper Motor Step Delays: Minimum Feasible (6ms)
Very short step delays can maximize the speed of the stepper motor but risk overshooting creating a misalignment in stepper position. The 6ms step delay proved to work in our system, however, it is significantly more prone to result in many different errors such as, misalignments, out of sync, and pieces hitting the rims of the bucket. Despite these errors, a successful run at 6ms is still possible.

Recommended (Stable) Operating Parameters:
PWM for DC Motor: 50 - 65%
At this setting, the system consistently performed without errors, ensuring that objects are sorted accurately and reliably. These specifically concern the speed settings for the DC motor which affects how quickly objects are transported along the conveyor belt. These speeds ensure items are able to be placed on the conveyor belt without falling over, and no pieces would fall off the end of the conveyor belt due to inertia.
Stepper Motor Step Delays: Optimal Balance (7ms)
Step delays that balance speed and accuracy prevent the motor from overshooting and reduce wear, ensuring longer component life and consistent performance. At 7ms minimum delay, the stepper bucket will never experience overshoots or missed steps.
Sensor Response Times: Balanced Accuracy
Optimizing sensor response times to balance speed and accuracy ensures reliable object detection without sacrificing system responsiveness.
Acceleration/Deceleration Ramps: Smoothed Profiles
Smoother acceleration and deceleration profiles for the stepper motor help in preventing mechanical strain and ensure the motor reaches and maintains desired positions accurately, especially when aligning the sorting bucket. For optimal accuracy 15 to 18 step increments (ex. 7ms to 25ms) proved to be the most reliable, never resulting in items hitting rims or misalignment errors. 

Testing and Calibration Procedures:
In order to ensure optimal performance of system functions, such as item categorization and stepper motor rotation, various tests and calibration procedures were performed throughout the development of the sorting system. 
Calibration of the ADC conversions were a critical aspect of this project, as this would ultimately determine the category of each of the 48 pieces passing through the system. In order to properly calibrate the ADC, the LCD was programmed to display the optimal ADC values as each item passes through the reflective sensor. Twelve pieces of each item type were sent through the reflective sensor and their ADC readings were recorded in an attempt to find the highest and lowest values for each piece type. Aluminum, being the most reflective piece, had the lowest ADC values, and thus only a maximum range was necessary and after the 12 trials, Aluminum pieces never had ADC values above 100. Steel was observed to have ADC values of no less than 300, averaging these results, it was determined that any piece with ADC values lower than 200 would fall into the aluminum category. Most steels were ranging from 380 to 580, and white plastic had values no less than 800, thus any items with ADC values between 201 and 700 would go into the steel category. The black and white plastics had ADC values that were much closer to each other, meaning a highly extensive analysis of ADC values were required. Average White plastic values ranged from 890 to 920, while average black plastic values ranged from 950 to 980. Therefore, it was determined that pieces with ADC values from 701 to 935 would be classified into the white plastic category, and all items higher than 935 would be classified as black plastic. These boundaries were chosen as they gave plenty of wiggle room for reading slightly higher or below the expected values, while also ensuring pieces would not fall into wrong categories. This arrangement of boundaries allowed for multiple runs with zero errors. The minimum to maximum values and final boundaries can be seen in Table 1 and Table 2 respectively.

Calibration Results Example:
Item Type,
Minimum - Maximum ADC Value;
Aluminum
30 - 100,
Steel
300 - 600,
White Plastic
800 - 920,
Black Plastic
950 - 990

Item Type
Final Maximum Acceptable Boundaries for ADC Values
Aluminum
<200,
Steel
<700,
White Plastic
<935,
Black Plastic
<1024


