package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    public void readPeriodic();
    public void writePeriodic();

    public void setTargetState(SwerveModuleState targetState); // Set the target state
    public SwerveModulePosition getModulePosition(double timestamp);

    public SwerveModuleState getCurrentState(); // This is the state according to the sensor input
}
