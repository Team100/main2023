package frc.robot.FRCLib.Motors;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class FRCVictorSPX {
    private int canID;
    private boolean inverted;
    private InvertType invertType;
    private boolean useInvertType;
    private VictorSPX motor;
    private FRCTalonSRX master;

    public FRCVictorSPX enableFollowing() {
        motor.follow(master.getMotor());
        return this;
    }

    public int getCanID() {
        return canID;
    }

    public void setCanID(int canID) {
        this.canID = canID;
    }

    public boolean isInverted() {
        return inverted;
    }
    
    public boolean isInvertedWithType() {
        return invertType != InvertType.None;
    }

    public InvertType getInvertType() {
        return this.invertType;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        this.invertType = InvertType.None;
        this.useInvertType = false;
    }

    public void setInverted(InvertType inverted) {
        this.invertType = inverted;
        this.inverted = false;
        this.useInvertType = true;
    }

    public VictorSPX getMotor() {
        return motor;
    }

    public void setMotor(VictorSPX motor) {
        this.motor = motor;
    }

    public FRCTalonSRX getMaster() {
        return master;
    }

    public void setMaster(FRCTalonSRX master) {
        this.master = master;
    }

    public FRCVictorSPX configure() {
        motor = new WPI_VictorSPX(this.getCanID());


        motor.configFactoryDefault();
        if (this.isInverted() || this.isInvertedWithType()) {
            if (this.useInvertType) motor.setInverted(this.invertType);
            else  motor.setInverted(this.isInverted());
            System.out.println("Configuring Inverted");
        }
        if (master != null) {
            motor.follow(master.motor);
            System.out.println("Configuring Master");
        }

        return this;
    }

    public static final class FRCVictorSPXBuilder {
        private int canID;
        private boolean inverted = false;
        private InvertType invertType = InvertType.None;
        private boolean useInvertType = false;
        private FRCTalonSRX master;

        public FRCVictorSPXBuilder(int canID) {
            this.canID = canID;

        }

        public static FRCVictorSPXBuilder aFRCVictorSPX(int canID) {
            return new FRCVictorSPXBuilder(canID);
        }

        public FRCVictorSPXBuilder withCanID(int canID) {
            this.canID = canID;
            return this;
        }

        public FRCVictorSPXBuilder withInverted(boolean inverted) {
            this.inverted = inverted;
            this.useInvertType = false;
            return this;
        }

        public FRCVictorSPXBuilder withInverted(InvertType inverted) {
            this.invertType = inverted;
            this.useInvertType = true;
            return this;
        }

        public FRCVictorSPXBuilder withMaster(FRCTalonSRX master) {
            this.master = master;
            return this;
        }

        public FRCVictorSPX build() {
            FRCVictorSPX fRCVictorSPX = new FRCVictorSPX();
            fRCVictorSPX.setCanID(canID);

            if (useInvertType) fRCVictorSPX.setInverted(invertType);
            else fRCVictorSPX.setInverted(inverted);
            if (master != null) fRCVictorSPX.setMaster(master);
            return fRCVictorSPX;
        }
    }
}
