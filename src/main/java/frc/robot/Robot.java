
package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.TelopDriveCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    public static final double PERIOD = .025;

    private final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    private final ShuffleboardTab robotTab;

    private final ShuffleboardContainer resetPositionLayout;
    private final GenericEntry xPositionEntry;
    private final GenericEntry yPositionEntry;
    private final GenericEntry thetaEntry;
    private final Command resetPoseCommand;

    public Robot()
    {
        super(PERIOD);
        Logger.recordMetadata("ProjectName", "Swerve Drivetrain");
        DriverStation.silenceJoystickConnectionWarning(true);

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
//            Logger.addDataReceiver(new WPILOGWriter());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();

        CommandScheduler.getInstance().setPeriod(.015);
        DrivetrainSubsystem.getInstance();

        IO.Initialize(IO.PrimaryDriverProfiles.Leo,IO.SecondaryDriverProfiles.KnollController);

        robotTab = Shuffleboard.getTab("Robot");
        resetPositionLayout = robotTab.getLayout("Reset Position", BuiltInLayouts.kList);

        xPositionEntry = resetPositionLayout.add("X Position", 0).getEntry();
        yPositionEntry = resetPositionLayout.add("Y Position", 0).getEntry();
        thetaEntry = resetPositionLayout.add("Theta", 0).getEntry();

        resetPoseCommand = new InstantCommand(()->{
            DrivetrainSubsystem.getInstance().resetPose(
                    new Pose2d(
                        xPositionEntry.getDouble(0),
                        yPositionEntry.getDouble(0),
                        Rotation2d.fromRadians(thetaEntry.getDouble(0))
                    )
            );
        });

        resetPositionLayout.add("Reset Pose Command", resetPoseCommand);
    }
    @Override
    public void robotInit() {
        new Trigger(IO.getButtonValue(Controls.resetGyro)).onTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyro()));
    }

    @Override
    public void robotPeriodic() {
        readPeriodic();
        CommandScheduler.getInstance().run();
        writePeriodic();
    }

    public void readPeriodic() {
        Logger.recordOutput("Reset Pose", new Pose2d(xPositionEntry.getDouble(0),yPositionEntry.getDouble(0),Rotation2d.fromRadians(thetaEntry.getDouble(0))));
        DrivetrainSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodic() {
        DrivetrainSubsystem.getInstance().writePeriodic();
    }
    
    @Override
    public void autonomousInit() {}
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new TelopDriveCommand());
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue != alliance)) {
            DrivetrainSubsystem.getInstance().switchAlliances();
        }
    }

    @Override
    public void simulationPeriodic() {
        DrivetrainSubsystem.getInstance().simulationPeriodic();
    }
}
