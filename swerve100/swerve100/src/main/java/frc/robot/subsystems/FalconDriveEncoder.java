package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;

public class FalconDriveEncoder  implements DriveEncoder {


    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FalconDriveEncoder");
        builder.addDoubleProperty("Speed", this::getRate, null);
    }
    
}
