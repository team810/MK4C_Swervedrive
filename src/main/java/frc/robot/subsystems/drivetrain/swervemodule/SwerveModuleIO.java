package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    /**
     * @param targetState Sets the target state for the module this should be called periodically. This state is applied in the writePeriodic function so this must be called before that if you want it to be applied
     */
    public void setTargetState(SwerveModuleState targetState);

    public void readPeriodic();
    /**
     * This should be called periodically after the swerve module state is set
     */
    public void writePeriodic();

    /**
     * This should be called in the drivetrain subsystem sim periodic function
     */
    public void moduleSim();

    /**
     * @param timestamp requires a timestamp that you want the measurement from to get this you can use the:
     *                        Utils.getCurrentTimeSeconds();
     * @return The current distance traveled by the wheel in meters, this should be plugged right into the swerve module position class
     */
    public double getPosition(double timestamp);

    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    public double getPosition();
    /**
     * @param timestamp requires a timestamp that you want the measurement from to get this you can use the:
     *                  Utils.getCurrentTimeSeconds();
     * @return The current velocity in meters per second
     */
    public double getVelocity(double timestamp);

    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    public double getVelocity();

    /**
     * @return Current acceleration in meters per second squared
     */
    public double getAcceleration(double timestamp);

    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    public double getAcceleration();

    /**
     * @return returns the voltage applied to the drive motor.
     */
    public double getDriveAppliedVoltage();

    /**
     * @return This is the current angle the wheel is facing in radians wrapped from -PI to PI
     */
    public double getTheta();

    /**
     * @return This is the current angular velocity of the wheel
     */
    public double getOmega();

    /**
     * @return the horizontal force created by the module
     */
    public double getForce();

    /**
     * @return Returns of the spinning wheel
     */
    public double getTorque();

    /**
     * @return returns the voltage applied to the steer motor.
     */
    public double getSteerAppliedVoltage();




}
