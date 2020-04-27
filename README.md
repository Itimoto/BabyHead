# BabyHead
Head-Tracking Camera. Almost VR, but not quite.

---
<a href="https://www.youtube.com/watch?v=eJpo-IbOa_U
" target="_blank"><img src="https://github.com/Itimoto/BabyHead/blob/master/misc/CameraThing.jpg" 
alt="Thumbnail for Video With Explanation" /></a>
>*The full shebang: We've got an All-In-One FPV Camera mounted to a Tilt motor on top of two Pan Servos hot-glued to each other. In the corner, you'll see the RF transceivers*

---
## Video Explanation:
<a href="https://www.youtube.com/watch?v=eJpo-IbOa_U
" target="_blank"><img src="https://github.com/Itimoto/BabyHead/blob/master/misc/Thumb.jpg" 
alt="Thumbnail for Video With Explanation" /></a>
>*The above video was made as part of a Scholarship Application. Please excuse the gratuitous Goodyear bling sprinkled through the video.*

At its core, the project consists of two parts: the *Goggles* and the *Head*. The 'Goggles' read information from the Accelerometer/Gyroscope Sensor, calculate its Euler Angles (i.e. the Angles that take it from its original 'inertial' position to its current 'body' position), then stream the Tilt and Pan values to the 'Head,' which then shifts the motors to the calculated position, matching the Goggle-User's head orientation.

## Concepts Explored:
- Euler Angles and Calculating Orientation
- Using Accelerometers & Gyroscopes
- Using SPI and other Serial Protocols to communicate across peripherals
- Using RF Transcievers to communicate between Processors

## Room For Improvement:
- Switch to Quaternion Orientation represenation
  - Euler Angles are subject to Gimbal Lock
  - However, this moves away from the intuitive aspect of BabyHead; need to learn more about Quaternions (i.e. Actually *understand* them) to move further
- Reduce latency
- Enable Online
  - This, however, introduces tons of latency. Food for thought, though
- Rename 'BabyHead' to anything *other* than 'BabyHead'
  - Hmm... **nah**
  - It's flawed, but the name stuck. The whole setup almost shrinks you; it's almost as if you were a *baby*
  - Yeah, no, it's *definitely* flawed, but I stand by it.

---
## Other Images of the Project:
*For all of our low-data users, these are clips from the video*

![Camera Close-Up](https://github.com/Itimoto/BabyHead/blob/master/misc/Camera.jpg)
>*Camera Close-up: Closer view of our AIO FPV Camera. Remember to get these on sale, y'all*

![Module Set to the Goggles](https://github.com/Itimoto/BabyHead/blob/master/misc/GoggleModule.jpg)
>*Goggle Module: The IMU (an Accelerometer + Gyroscope Combo) is attached to an Arduino which broadcasts the sensor data through the RF transciever. The module's taped to an FPV Headset*

![View from Inside FPV Goggles](https://github.com/Itimoto/BabyHead/blob/master/misc/GoggleView.jpg)
>*Goggle View: There's no need to splurge on FPV gear. Sometimes, gambling with Alibaba is worth it. Sometimes.*

![Tasteful Soldering Shot](https://github.com/Itimoto/BabyHead/blob/master/misc/SolderingIron.jpg)
>*For more details, check out the video.*
