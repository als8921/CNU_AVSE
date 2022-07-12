# KABOAT2022

# LOS Guidance.py
* Line Of Sight Guidance
### Subscribed Data
- **/WayPoint** (Float64MultiArray)
- **/GPSData** (Float64MultiArray)

### Published Data
- **/Psi_D** (Float32)

# PWM Control

### Subscribed Data
- **/IMUData** (Float32)
- **/PSI_D** (Float32)

### Published Data
- **/PWM** (Float64MultiArray)

# PWM Plot
using Matplotlib plot LPWM, RPWM, Psi, Psi_D 

# SimulLidarPlot
* Polar
* Flat
