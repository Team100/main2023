package org.team100.frc2023;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.team100.frc2023.autonomous.Autonomous;
import org.team100.frc2023.autonomous.DriveToAprilTag;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveManually;
import org.team100.frc2023.commands.DriveMedium;
import org.team100.frc2023.commands.DriveRotation;
import org.team100.frc2023.commands.DriveSlow;
import org.team100.frc2023.commands.DriveWithHeading;
import org.team100.frc2023.commands.GripManually;
import org.team100.frc2023.commands.ResetPose;
import org.team100.frc2023.commands.ResetRotation;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.ManualArm;
import org.team100.frc2023.commands.Arm.Oscillate;
import org.team100.frc2023.commands.Arm.SetConeMode;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Close;
import org.team100.frc2023.commands.Manipulator.CloseSlow;
import org.team100.frc2023.commands.Manipulator.Home;
import org.team100.frc2023.commands.Manipulator.Open;
import org.team100.frc2023.commands.Retro.DriveToRetroReflectiveTape;
import org.team100.frc2023.commands.Retro.LedOn;
import org.team100.frc2023.control.Control;
import org.team100.frc2023.control.DualXboxControl;
import org.team100.frc2023.control.JoystickControl;
import org.team100.frc2023.retro.Illuminator;
import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.Manipulator;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

@SuppressWarnings("unused")
public class RobotContainer implements Sendable {

    // CONFIG

    public DriverStation.Alliance m_alliance;

    // SUBSYSTEMS
    public final SwerveDriveSubsystem m_robotDrive;
    private final Manipulator manipulator;
    private final ArmController armController;
    private final AHRSClass ahrsclass;

    // CONTROL
    private final Control control;

    // COMMANDS
    private final AutoLevel autoLevel;
    private final DriveManually driveManually;
    private final GripManually gripManually;
    private final ManualArm manualArm;

    private final DriveWithHeading driveWithHeading;
    private final DriveRotation driveRotation;
    private final Defense defense;
    private final ResetRotation resetRotation0;
    private final ResetRotation resetRotation180;

    File myObj;
    private final FileWriter myWriter;

    private final DriveToAprilTag driveToSubstation, driveToLeftGrid, driveToCenterGrid, driveToRightGrid;

    private final ArmTrajectory armHigh;
    private final ArmTrajectory armSafe;
    private final ArmTrajectory armSubstation;
    private final ArmTrajectory armLow;
    private final ArmTrajectory armSafeBack;
    private final ArmTrajectory armToSub;
    private final ArmTrajectory armMid;
    private final ArmTrajectory armSafeWaypoint;
    private final Oscillate oscillate;
    // private final ArmTrajectory armSafeWaypoint;

    private final Illuminator illuminator;

    private final LedOn ledOn;
    private final DriveToRetroReflectiveTape retroTape;

    private final SetCubeMode setCubeMode;
    private final SetConeMode setConeMode;

    private final Home homeCommand;
    private final Close closeCommand;
    private final Open openCommand;
    private final CloseSlow closeSlowCommand;

    private final DriveSlow driveSlow;

    private final MoveConeWidth moveConeWidthLeft;
    private final MoveConeWidth moveConeWidthRight;

    public final static Field2d m_field = new Field2d();

    public final RumbleOn rumbleOn;

    public final Rotate rotateCommand;

    public static boolean enabled = false;

    public DriveMedium driveMediumCommand;

    public double m_routine = -1;

    public RobotContainer(DriverStation.Alliance alliance) throws IOException {
        // THIS IS BABY MODE
        // final double kDriveCurrentLimit = 20;
        // 60 amps is the maximum maximum
        final double kDriveCurrentLimit = 60;
        ahrsclass = new AHRSClass();
        manipulator = new Manipulator();
        armController = new ArmController();
        illuminator = new Illuminator();

        // // NEW CONTROL
        // control = new DualXboxControl();
        control = new JoystickControl();

        myObj = new File("/home/lvuser/logs.txt");
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter("/home/lvuser/logs.txt", true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        myWriter = fileWriter;

        // m_alliance = DriverStation.getAlliance();
        m_alliance = alliance;

        m_robotDrive = new SwerveDriveSubsystem(m_alliance, kDriveCurrentLimit, ahrsclass, control);

        if (m_alliance == DriverStation.Alliance.Blue) {
            // driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(6, 0.95, .55,
            // control::goalOffset, m_robotDrive, ahrsclass);
            driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(6, 1.25, 0, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);

            driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(7, 0.95, .55, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
            driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(8, 0.95, .55, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
            driveToSubstation = DriveToAprilTag.newDriveToAprilTag(4, 0.53, -0.749, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
        } else {
            driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(1, 0.95, .55, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
            driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(2, 0.95, .55, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
            driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(3, 0.95, .55, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);
            // driveToSubstation = DriveToAprilTag.newDriveToAprilTag(5, 0.53, -0.749,
            // control::goalOffset, m_robotDrive, ahrsclass);
            driveToSubstation = DriveToAprilTag.newDriveToAprilTag(5, 0.9, -0.72, control::goalOffset, m_robotDrive,
                    ahrsclass, () -> 0.0);

        }

        // if (m_alliance == DriverStation.Alliance.Blue) {
        // // driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(6, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass);
        // driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(6, 1.25, 0,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());

        // driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(7, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(8, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // driveToSubstation = DriveToAprilTag.newDriveToAprilTag(4, 0.53, -0.749,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // } else {
        // driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(1, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(2, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(3, 0.95, .55,
        // control::goalOffset, m_robotDrive, ahrsclass, () ->
        // manipulator.getGamePieceOffset());
        // // driveToSubstation = DriveToAprilTag.newDriveToAprilTag(5, 0.53, -0.749,
        // control::goalOffset, m_robotDrive, ahrsclass);
        // driveToSubstation = DriveToAprilTag.newDriveToAprilTag(5, 0.9, -0.72,
        // control::goalOffset, m_robotDrive, ahrsclass,() ->
        // manipulator.getGamePieceOffset());

        // }

        armHigh = new ArmTrajectory(ArmPosition.HIGH, armController);

        armSafe = new ArmTrajectory(ArmPosition.SAFE, armController);

        armSubstation = new ArmTrajectory(ArmPosition.SUB, armController);

        oscillate = new Oscillate(armController);

        ResetRotation resetRotation = new ResetRotation(m_robotDrive, new Rotation2d());
        autoLevel = new AutoLevel(false, m_robotDrive, ahrsclass);

        ResetPose resetPose = new ResetPose(m_robotDrive, 0, 0, 0);

        homeCommand = new Home(manipulator);

        openCommand = new Open(manipulator);

        closeCommand = new Close(manipulator);

        setCubeMode = new SetCubeMode(armController, m_robotDrive);

        setConeMode = new SetConeMode(armController, m_robotDrive);

        driveSlow = new DriveSlow(m_robotDrive, control);

        resetRotation0 = new ResetRotation(m_robotDrive, new Rotation2d(0));

        resetRotation180 = new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180));

        armLow = new ArmTrajectory(ArmPosition.MID, armController);

        armToSub = new ArmTrajectory(ArmPosition.SUBTOCUBE, armController);

        armMid = new ArmTrajectory(ArmPosition.LOW, armController);

        retroTape = new DriveToRetroReflectiveTape(m_robotDrive);

        closeSlowCommand = new CloseSlow(manipulator);

        rotateCommand = new Rotate(m_robotDrive, 0);

        armSafeWaypoint = new ArmTrajectory(ArmPosition.SAFEWAYPOINT, armController);

        driveWithHeading = new DriveWithHeading(
                m_robotDrive,
                control::xSpeed,
                control::ySpeed,
                control::desiredRotation,
                control::rotSpeed,
                "",
                ahrsclass);

        driveRotation = new DriveRotation(m_robotDrive, control::rotSpeed);

        defense = new Defense(m_robotDrive);

        manualArm = new ManualArm(armController, control);

        armSafeBack = new ArmTrajectory(ArmPosition.SAFEBACK, armController);

        ledOn = new LedOn(illuminator);

        rumbleOn = new RumbleOn(control);

        driveMediumCommand = new DriveMedium(m_robotDrive, control);

        moveConeWidthLeft = new MoveConeWidth(m_robotDrive, 1);
        moveConeWidthRight = new MoveConeWidth(m_robotDrive, -1);

        // control.autoLevel(autoLevel);
        control.driveToLeftGrid(driveToLeftGrid);
        control.driveToCenterGrid(driveToCenterGrid);
        control.driveToRightGrid(driveToRightGrid);
        control.driveToSubstation(driveToSubstation);
        control.defense(defense);

        control.resetRotation0(resetRotation0);
        control.resetRotation180(resetRotation180);

        control.armHigh(armHigh);
        control.armSafe(armSafe);
        // control.armSubstation(armSubstation);

        // control.open(openCommand);
        control.close(closeCommand);
        control.home(homeCommand);

        control.coneMode(setConeMode);
        control.cubeMode(setCubeMode);

        control.driveSlow(driveSlow);

        control.armSubstation(armSubstation);

        control.armLow(armLow);

        control.armSafeBack(armSafeBack);

        control.armToSub(armToSub);

        // control.armMid(armMid);

        control.resetPose(resetPose);

        control.ledOn(ledOn);

        control.tapeDetect(retroTape);

        control.rumbleTrigger(rumbleOn);

        control.closeSlow(closeSlowCommand);

        control.rotate0(rotateCommand);

        control.driveMedium(driveMediumCommand);

        // control.armSubSafe(armSubSafe);
        control.armSafe(armSafe);

        control.safeWaypoint(armSafeWaypoint);

        control.oscillate(oscillate);

        // control.armSafeSequential(armSafeWaypoint, armSafe);

        control.moveConeWidthLeft(moveConeWidthLeft);
        control.moveConeWidthRight(moveConeWidthRight);

        // DEFAULT COMMANDS
        // Controller 0 right => cartesian, left => rotation
        driveManually = new DriveManually(
                control::xSpeed,
                control::ySpeed,
                control::rotSpeed,
                m_robotDrive);

        ///////////////////////////
        // DRIVE OPTIONS
        //
        // SHOW mode
        // m_robotDrive.setDefaultCommand(driveManually);

        // NORMAL mode
        m_robotDrive.setDefaultCommand(driveWithHeading);


        /////////////////////////
        // MANIPULATOR

        // Controller 1 triggers => manipulator open/close
        gripManually = new GripManually(
                control::openSpeed,
                control::closeSpeed,
                manipulator);

        // manipulator.setDefaultCommand(gripManually);

        manipulator.setDefaultCommand(new RunCommand(() -> {
            manipulator.pinch(0);
        }, manipulator));

        armController.setDefaultCommand(manualArm);

        SmartDashboard.putData("Robot Container", this);
    }

    public Command getAutonomousCommand2(int routine, boolean isBlueAlliance) {
        // return new SequentialCommandGroup(
        // new IshanAutonomous(m_robotDrive),
        // new IshanAutonomous(m_robotDrive)
        // );

        return new Autonomous(m_robotDrive, armController, manipulator, ahrsclass, routine, isBlueAlliance);

        // return new SanjanAutonomous(AutonSelect.BLUE1, m_robotDrive, armController,
        // manipulator);
        // return new Autonomous(Aut-onSelect.BLUE2, AutonGamePiece.CONE, m_robotDrive,
        // armController, manipulator);
    }

    public void runTest() {
        XboxController controller0 = new XboxController(0);
        System.out.printf(
                "name: %s   left X: %5.2f   left Y: %5.2f   right X: %5.2f   right Y: %5.2f   left T: %5.2f   right T: %5.2f\n",
                DriverStation.getJoystickName(0),
                // DriverStation.getStickAxis(0, 0),
                controller0.getLeftX(), // 0 = GP right X, -0.66 to 0.83
                controller0.getLeftY(), // 1 = GP right Y, -0.64 to 0.64
                controller0.getRightX(), // 4 = GP left X, -0.7 to 0.8
                controller0.getRightY(), // 5
                controller0.getLeftTriggerAxis(), // 2 = GP left Y, -0.64 to 0.6
                controller0.getRightTriggerAxis() // 3
        );
    }

    public void runTest2() {

        XboxController controller0 = new XboxController(0);
        boolean rearLeft = controller0.getAButton();
        boolean rearRight = controller0.getBButton();
        boolean frontLeft = controller0.getXButton();
        boolean frontRight = controller0.getYButton();
        double driveControl = controller0.getLeftY();
        double turnControl = controller0.getLeftX();
        double[][] desiredOutputs = {
                { frontLeft ? driveControl : 0, frontLeft ? turnControl : 0 },
                { frontRight ? driveControl : 0, frontRight ? turnControl : 0 },
                { rearLeft ? driveControl : 0, rearLeft ? turnControl : 0 },
                { rearRight ? driveControl : 0, rearRight ? turnControl : 0 }
        };
        m_robotDrive.test(desiredOutputs, myWriter);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("container");
        builder.addDoubleProperty("theta controller error", () -> m_robotDrive.thetaController.getPositionError(),
                null);
        builder.addDoubleProperty("x controller error", () -> m_robotDrive.xController.getPositionError(), null);
        builder.addDoubleProperty("y controller error", () -> m_robotDrive.yController.getPositionError(), null);
        builder.addBooleanProperty("Is Blue Alliance", () -> isBlueAlliance(), null);
        builder.addDoubleProperty("Routine", () -> getRoutine(), null);

    }

    public void ledStart() {
        m_robotDrive.indicator.orange();
    }

    public void ledStop() {
        m_robotDrive.indicator.close();
    }

    public static boolean isEnabled() {
        return enabled;
    }

    public boolean isBlueAlliance() {
        if (m_alliance == DriverStation.Alliance.Blue) {
            return true;
        }
        return false;

    }

    public double getRoutine() {
        return m_routine;
    }
}
