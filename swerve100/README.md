# swerve100

This is the code joel wrote to make the old team 100 andymark swerve base work.  It started with the WPILib SwerveDriveController
example, and I added shims for the various hardware used, e.g. VictorSRX etc.

There's a fork of it where all the real work is happening.

The original swerve base uses this hardware:

* CIM drive motor with Talon SRX controllers 
* PG71 steering motor with Talon SRX controllers 
* Drive encoder are CIMcoders, plugged into the Talon SRX, which seems like maybe it is broken somehow?
* Steering encoder are both quad encoders (which we ignore) and MA3 encoders.  the Talon doesn't seem to understand the MA3 correctly.

The second swerve base uses this hardware:

* Falcon 500 drive motors, thus Falcon FX CAN-based control
* PG71 steering motors, using Victor SP PWM controllers
* Drive encoder is the Falcon integrated one.
* Steering encoder is the same MA3 as above, using the RIO analog input.
