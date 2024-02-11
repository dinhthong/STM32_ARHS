# Brief
Keil C IDE
Calculate attitutes of robot.
Read GY-86 MPU6050, HMC5883L, MS5611. roll, pitch, yaw and theirs respective rates, altitude

# How to run
## Hardware connection
- I2C MPU6050: PB6, PB7
- HMC5883: IO I2C WriteBytes. IO I2C2. PB10, PB11.
## Software
- Watch rpy[3]

# To do
- Currently the result is not consistent, need verification
Serial Print for debugging.
Write more universal code to implement other AHRS algorithms.
# References

