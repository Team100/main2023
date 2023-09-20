package org.team100.frc2023.subsystems.arm;

import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motor.FRCNEO;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.PIDController100;
import org.team100.lib.controller.PidGains;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.arm.ArmAngles;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Arm servo. Subsystem runs *all the time*, commands feed it references.
 */
public class ArmSubsystem extends Subsystem implements ArmInterface {
    private static class Noop extends Subsystem implements ArmInterface {

        @Override
        public Subsystem subsystem() {
            return this;
        }

		@Override
		public boolean getCubeMode() {
			return false;
		}

		@Override
		public void setCubeMode(boolean b) {			
		}

        @Override
        public void setReference(ArmAngles reference) {
            
        }

        @Override
        public ArmAngles getMeasurement() {
            return new ArmAngles(0,0);
        }

        @Override
        public void setControlNormal() {
        }

        @Override
        public void setControlSafe() {            
        }

        @Override
        public void close() {            
        }

        @Override
        public boolean getReferenceUsed(){
            return true;
        }

        @Override
        public void setReferenceUsed(boolean bool) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void resetFilters(){

        }

        @Override
        public void setControlOscillate() {
            // TODO Auto-generated method stub
            
        }



    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public ArmInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                    return new ArmSubsystem();
                default:
                    return new Noop();
            }
        }
    }

    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
        public double filterTimeConstantS = 0.1; // TODO: tune the time constant
        public double filterPeriodS = 0.02;
        public double safeP = 2.5;
        public double safeI = 0;
        public double safeD = 0;
        
        public PidGains safeGains = new PidGains(2.5, 0, 0);
        public PidGains normalLowerGains = new PidGains(2, 0, 0.1);
        public PidGains normalUpperGains = new PidGains(2, 0, 0.05);
        public PidGains oscillateGains = new PidGains(1, 0, 0);

        public double normalLowerP = 2; //0.3
        public double normalLowerI = 0;
        public double normalLowerD = 0.1;
        public double normalUpperP = 2;
        public double normalUpperI = 0;
        public double normalUpperD = 0.05;
        public double tolerance = 0.1;
    }

    // TODO: do something with this
    public static class ArmState {
        private final State100 m_lower;
        private final State100 m_upper;

        public ArmState(State100 lower, State100 upper) {
            m_lower = lower;
            m_upper = upper;
        }

        public State100 lower() {
            return m_lower;
        }

        public State100 upper() {
            return m_upper;
        }
    }

    private final Config m_config = new Config();

    /** Kinematics in meters. Coordinates are x up, y forward. */
    private final ArmKinematics m_armKinematicsM;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final PIDController100 m_lowerController;
    private final PIDController100 m_upperController;
    private final FRCNEO lowerArmMotor;
    private final FRCNEO upperArmMotor;
    private final AnalogInput lowerArmInput;
    private final AnalogInput upperArmInput;
    private final AnalogEncoder lowerArmEncoder;
    private final AnalogEncoder upperArmEncoder;
    private ArmAngles m_measurment;
    private ArmAngles referencePrint;
    private double m_u1;
    private double m_u2;
    private double returnLowerArmFilter = 0;
    private double returnUpperArmFilter = 0;
    public boolean referenceUsed = false;
    private double count = 0;
    // TODO: move this somewhere else
    private boolean cubeMode = true;
    public void setCubeMode(boolean mode) {
        cubeMode = mode;
    }
    @Override
    public boolean getCubeMode() {
        return cubeMode;
    }
    private ArmAngles m_reference;

    private ArmSubsystem() {
        m_armKinematicsM = new ArmKinematics(m_config.kLowerArmLengthM, m_config.kUpperArmLengthM);
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_lowerController = new PIDController100(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController = new PIDController100(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        m_lowerController.setTolerance(m_config.tolerance);
        m_upperController.setTolerance(m_config.tolerance);
        m_measurment = new ArmAngles();
        referencePrint = new ArmAngles();
        lowerArmMotor = new FRCNEO.FRCNEOBuilder(43)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(40)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .build();

        upperArmMotor = new FRCNEO.FRCNEOBuilder(42)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(40)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .withForwardSoftLimitEnabled(false)
                .build();

        lowerArmInput = new AnalogInput(6);
        lowerArmEncoder = new AnalogEncoder(lowerArmInput);
        upperArmInput = new AnalogInput(5);
        upperArmEncoder = new AnalogEncoder(upperArmInput);

        // m_lowerMeasurementFilter.calculate(getLowerArm());
        // m_upperMeasurementFilter.calculate(getUpperArm());

        for(int i = 0; i < 15; i++){
            double x = m_lowerMeasurementFilter.calculate(getLowerArm());
            double y = m_upperMeasurementFilter.calculate(getUpperArm());

            //  System.out.println("LOWER: " + x);
            // System.out.println("UPPER: " + y);
           
            // System.out.println("LOWER RAW: " + getLowerArm());
            // System.out.println("UPPER RAW: " + getUpperArm());
        }
        m_reference = getMeasurement();

        SmartDashboard.putData("Arm Subsystem", this);
    }

    public void setReferenceUsed(boolean bool){
        referenceUsed = bool;
    }

    public boolean getReferenceUsed(boolean bool){
        return referenceUsed;
    }
    @Override  
    public void periodic() {
        ArmAngles measurement = getMeasurement();
            double u1 = m_lowerController.calculate(measurement.th1, m_reference.th1);
            double u2 = m_upperController.calculate(measurement.th2, m_reference.th2);
            referencePrint = m_reference;
            
            m_measurment = measurement;
           
            // System.out.println(u1);
            // System.out.println(u2);
    
            m_u1 = u1;
            m_u2 = u2;
    
            lowerArmMotor.set((u1));
            upperArmMotor.set(u2);
        
    }

    /**
     * Update the reference to track. This might come from a trajectory sampler or
     * from manual control.
     */
    public void setReference(ArmAngles reference) {
        m_reference = reference;
    }

    /** Measure the arm position, smoothed with single-pole IIR low-pass filter. */
    public ArmAngles getMeasurement() {


        returnLowerArmFilter =  m_lowerMeasurementFilter.calculate(getLowerArm());
        returnUpperArmFilter = m_upperMeasurementFilter.calculate(getUpperArm());


       

        // returnLowerArmFilter =  m_lowerMeasurementFilter.calculate(getLowerArm());
        // returnUpperArmFilter = m_upperMeasurementFilter.calculate(getUpperArm());

        return new ArmAngles(
                returnLowerArmFilter,
                returnUpperArmFilter);

        // return new ArmAngles(
        //         getLowerArm(),
        //         getUpperArm());
    }

    public void setControlOscillate(){
        m_upperController.setPID(m_config.oscillateGains);
    }
    
    public void setControlNormal() {
        m_lowerController.setPID(m_config.normalLowerGains);
        m_upperController.setPID(m_config.normalUpperGains);
    }

    /**
     * Safe control parameters are gentler, intended to keep from crashing into the
     * safe-position hard-stop.
     */
    public void setControlSafe() {
        m_lowerController.setPID(m_config.safeGains);
        m_upperController.setPID(m_config.safeGains);
    }


    

    
    public void close() {
        lowerArmMotor.close();
        upperArmMotor.close();
        lowerArmEncoder.close();
        upperArmEncoder.close();
        lowerArmInput.close();
        upperArmInput.close();
    }

    ///////////////////////////////////////////////////////////////////////////////

    /** Stop the motor at the soft stop. */
    private double soften(double x) {
        if (softBand(x))
            return 0;
        return x;
    }

    /** Should the motor stop? */
    private boolean softBand(double x) {
        if (getLowerArm() <= m_config.softStop && x < 0)
            return false;
        return true;
    }

    /** Lower arm angle (in radians) 0 up, positive forward. */
    private double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArm() {
        double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 360;
        return x * Math.PI / 180;
    }

    private double getUpperArmDegrees() {
        return getUpperArm() * 180 / Math.PI;
    }

    private double getLowerArmDegrees() {
        return getLowerArm() * 180 / Math.PI;
    }

    private ArmAngles getm_Measurment(){
        if(m_measurment == null){
            return new ArmAngles();
        } else{
            return m_measurment;
        }
    }

    private ArmAngles getReferencePrint(){
        if(referencePrint == null){
            return new ArmAngles();
        } else{
            return referencePrint;
        }
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ARM X", () -> m_armKinematicsM.forward(getMeasurement()).getX(), null);
        builder.addDoubleProperty("ARM Y", () -> m_armKinematicsM.forward(getMeasurement()).getY(), null);
        builder.addDoubleProperty("Upper Arm Absolute Angle", () -> upperArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Lower Arm Absolute Angle", () -> lowerArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Upper Arm Absolute Radians", () -> getUpperArm(), null);
        builder.addDoubleProperty("Lower Arm Absolute Rasians", () -> getLowerArm(), null);
        builder.addDoubleProperty("Upper Arm Absolute Degrees", () -> getUpperArmDegrees(), null);
        builder.addDoubleProperty("Lower Arm Absolute Degrees", () -> getLowerArmDegrees(), null);
        builder.addBooleanProperty("Cube Mode", () -> cubeMode, null);
        builder.addDoubleProperty("Upper Angle Setpoint", () -> m_reference.th2, null);
        builder.addDoubleProperty("Lower Angle Setpoint", () -> m_reference.th1, null);
        builder.addDoubleProperty("U1", () -> m_u1, null);
        builder.addDoubleProperty("U2", () -> m_u2, null);

        builder.addDoubleProperty("Measurment Proximal", () -> getm_Measurment().th1, null);
        builder.addDoubleProperty("Measurment Distal", () -> getm_Measurment().th2, null);
        builder.addDoubleProperty("Reference Proximal", () -> getReferencePrint().th1, null);
        builder.addDoubleProperty("Reference Distal", () -> getReferencePrint().th2, null);
       
        builder.addDoubleProperty("Lower Arm Filter", () -> returnLowerArmFilter, null);
        builder.addDoubleProperty("Upper Arm Filter", () -> returnUpperArmFilter, null);


    }
    @Override
    public boolean getReferenceUsed() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void resetFilters() {
        // for(int i = 0; i < 10; i++){
                // m_lowerMeasurementFilter.calculate(getLowerArm());
                // m_upperMeasurementFilter.calculate(getUpperArm());
        //     System.out.println("LOWER: " + returnLowerArmFilter);
        //     System.out.println("UPPER: " + returnUpperArmFilter);
           
        //     System.out.println("LOWER RAW: " + getLowerArm());
        //     System.out.println("UPPER RAW: " + getUpperArm());
        // }
        
    }
}