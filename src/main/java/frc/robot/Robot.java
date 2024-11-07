
package frc.robot;


import com.ctre.phoenix6.Orchestra;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    public static final double PERIOD = .025;
    private final DrivetrainSubsystem drivetrain;

    private final Orchestra orchestra = new Orchestra();

    public Robot()
    {
        super(defaultPeriodSecs);
        
        Logger.recordMetadata("ProjectName", "Swerve Drivetrain");
        DriverStation.silenceJoystickConnectionWarning(true);

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        CommandScheduler.getInstance().setPeriod(.015);

        // Init for subsystems
        drivetrain = new DrivetrainSubsystem();


    }
    @Override
    public void robotInit() {
    }
    
    
    @Override
    public void robotPeriodic() {
        readPeriodic();
        CommandScheduler.getInstance().run();
        writePeriodic();
    }

    public void readPeriodic() {
        if (Robot.isSimulation())
        {
            drivetrain.simulatePeriodic();
        }

        drivetrain.readPeriodic();
    }

    public void writePeriodic() {
        drivetrain.writePeriodic();
    }
    
    @Override
    public void autonomousInit() {}
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit() {}
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }
}
