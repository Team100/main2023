package org.team100.frc2023.commands.Retro;

import java.io.IOException;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.frc2023.localization.Tapes;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToRetroReflectiveTape extends CommandBase {
  /** Creates a new DriveToRetroReflectiveTape. */


  private final ObjectMapper object_mapper;
  private final RawSubscriber tapeSubscriber;
  
  Tapes tapes;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController xController;

  SwerveDriveSubsystem m_robotDrive;
  boolean done = false;
  boolean firstRun = false;

  double xMeasurment = 0;
  double yMeasurment = 0;

  LinearFilter xFilter = LinearFilter.singlePoleIIR(0.06, 0.02); 
  LinearFilter yFilter = LinearFilter.singlePoleIIR(0.06, 0.02); 


  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  DoublePublisher setpointX = inst.getTable("Retro Tape").getDoubleTopic("Setpoint X").publish();
  DoublePublisher setpointY = inst.getTable("Retro Tape").getDoubleTopic("Setpoint Y").publish();
  
  DoublePublisher measurmentX = inst.getTable("Retro Tape").getDoubleTopic("Measurment X").publish();
  DoublePublisher measurmentY = inst.getTable("Retro Tape").getDoubleTopic("Measurment Y").publish();

  DoublePublisher errorX = inst.getTable("Retro Tape").getDoubleTopic("Error X").publish();
  DoublePublisher errorY = inst.getTable("Retro Tape").getDoubleTopic("Error Y").publish();


  DoublePublisher tagView = inst.getTable("Retro Tape").getDoubleTopic("Tag View").publish();


  DoublePublisher xOutputPub = inst.getTable("Retro Tape").getDoubleTopic("X Ouput View").publish();
  DoublePublisher yOutputPub = inst.getTable("Retro Tape").getDoubleTopic("Y Output View").publish();

  public DriveToRetroReflectiveTape(SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable tapeTable = inst.getTable("RetroVision");

    m_robotDrive = robotDrive;

    

    yController = new ProfiledPIDController(1, 0, 0, new Constraints(0.25, 0.5) );
    xController = new ProfiledPIDController(1.2, 0, 0, new Constraints(0.25, 0.5) );

    xController.setTolerance(0.03);
    yController.setTolerance(0.03);

    tapeSubscriber = tapeTable.getRawTopic("tapes").subscribe("raw", new byte[0]);
    object_mapper = new ObjectMapper(new MessagePackFactory());
    // tapes = new Tapes();




  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    firstRun = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tapes tape = object_mapper.readValue(event.valueData.value.getRaw(), Blips.class);
    

    try {
      byte[] data = tapeSubscriber.get();
      tapes = object_mapper.readValue(data, Tapes.class);

      System.out.println(data.length);
      
      
      
      if(tapes.tapes.size() > 0){

        yMeasurment = yFilter.calculate(-tapes.tapes.get(0).pose_t[1]);
        xMeasurment = xFilter.calculate(-tapes.tapes.get(0).pose_t[0]);

        // yMeasurment = -tapes.tapes.get(0).pose_t[1];
        // xMeasurment = -tapes.tapes.get(0).pose_t[0];

        if(firstRun == true){
          yController.reset(yMeasurment);
          xController.reset(xMeasurment);
          firstRun = false;
        }
        double yOutput = yController.calculate(yMeasurment, 0);
        double xOutput = xController.calculate(xMeasurment, -0.6);

        // xOutput = MathUtil.clamp(xOutput, -0.15, 0.15);
        // yOutput = MathUtil.clamp(yOutput, -0.15, 0.15);

        measurmentX.set(xMeasurment);
        measurmentY.set(yMeasurment);

        tagView.set(0);

        System.out.println("*************************************************************" + xOutput);
        System.out.println("*************************************************************" + yOutput);

        yOutputPub.set(yOutput);
        xOutputPub.set(xOutput);

        // xOutput = MathUtil.applyDeadband(xOutput, 0.01);
        // yOutput = MathUtil.applyDeadband(yOutput, 0.01);


        errorX.set(xController.getPositionError());
        errorY.set(yController.getPositionError());
        m_robotDrive.driveTape(xOutput, yOutput, 0, false);
      } else {
        m_robotDrive.driveTape(0, 0, 0, false);
        tagView.set(2);
      
      }

      setpointX.set(xController.getSetpoint().position);
      setpointY.set(yController.getSetpoint().position);

      

    } catch (IOException e) {
      e.printStackTrace();
    }

    // byte[] data = tapeSubscriber.get();

    // System.out.println("**************************************************************************************************************" + data.length);
    

    





  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
