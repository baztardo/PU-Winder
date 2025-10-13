# PU-Winder
Automated Guitar Pick up winder



<h3>The Goal is make an automated guitar pick up winder.</h3>
<p>The Z Spindle to home, then the Traverse carriage home's hit the home switch and back off,</p>
<p>Then it move to it home position approx. 20mm and waits to start</p>

<p>The Traverse carriage should sync to the Spindles(Bobbin) RPM to ensure smooth even layers</p>
<p>of the copper wire.</p>
<p>The encoder A/B should track the pulse to ensure the spindle is turn CW/CCW and its movements</p>
<p>The Z should keep track of each revolution as it only pulse's once per/rev</p>
<p>All counts should be able to go up and down depending on the rotation CW/CCW</p>
<p>The spindle once started should run until it hits the winding target then stop.</p>




<h3>Hardware:</h3>
<p>24v 6a Power supply</p>
<p>Target RPM 1000, 1500</p>
<p>TMC2209 drivers</p>
<p>Encoder PPR 360 Quadrature A, B, Z, 5v, Gnd -- Resolution 1440 lines/pulse's</p>

<p>Nema 17 Stepper Motor 1.8' 200 Steps -- 8 Microstepping -- 1600 Microsteps </p>
<p></p>
<p>I2C 4 Line 20 charactor LCD  2004A</p>


---------------------------------------------------------------------------------

<p><h3>MPU's</h3></p>
<p>I Would like to use an STM32 but I not familiar whit it.</p>
<p></p>
<p>All of my past development has been on the Arduino</p>
<p>I have decided to move the Pico using the VSCode IDE and the Pico-sdk </p>
<p>I have chosen the SKR-Pico control board since it has most of what I need on board in a small package</p>

---------------------------------------------------------------------------------
<p>Bobbin 7 mm Width</p>
<p>copper winding Wire 43 AWG -- 0.0024" -- 0.0635 mm</p>
<p>7mm / 0.0635mm = 110.2362 Turns per layer of wire</p>
Single coil is approx 5000 Turns approx.
P90 coil is 10,000 turns approx.</P>


Reference:
<p>https://cdn-shop.adafruit.com/datasheets/TC2004A-01.pdf</p>
<p>https://github.com/bigtreetech/SKR-Pico/blob/master/Hardware/BTT%20SKR%20Pico%20V1.0-PIN.pdf</p>
<p>https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf</p>


---------------------------------------------------------------------------------
New Photos
![20251013_134018](https://github.com/user-attachments/assets/634ed18b-aec4-4439-87cd-a1abc3a5f9f9)

Some Prototype Photo's

<img src="https://user-images.githubusercontent.com/99566898/153733756-61c28bf5-6e95-42fd-b2d8-9229b98b4e05.jpg" width=40% height=40%>
<img src="https://user-images.githubusercontent.com/99566898/153733760-ed26199b-2017-4dbd-aae8-3ed6b6aec183.jpg" width=40% height=40%>
<img src="https://user-images.githubusercontent.com/99566898/153733767-a41110f0-5c6c-428e-bcec-da48494e5bcf.jpg" width=40% height=40%>
<img src="https://user-images.githubusercontent.com/99566898/153733944-d830ba08-0776-47ef-a721-b181705602b7.jpg" width=40% height=40%>

<img src="https://user-images.githubusercontent.com/99566898/153734277-ac632920-4db7-4523-b358-a69a652eae81.jpg" width=40% height=40%>
