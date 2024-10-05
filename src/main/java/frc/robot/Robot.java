
package frc.robot;

import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.SwerveModuleKrakenNeo;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot
{
    SwerveModuleKrakenNeo module = new SwerveModuleKrakenNeo(1,2,3, SwerveModule.FrontLeft);
    @Override
    public void robotInit() {
    }
    
    
    @Override
    public void robotPeriodic() {}
    
    
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
    public void testInit() {}
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void simulationInit() {}
    
    
    @Override
    public void simulationPeriodic() {}
}
