package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class AdvancedSwerveModuleState {
    public AdvanceSwerveModuleInstant previousState;
    public AdvanceSwerveModuleInstant currentState;

    public AdvancedSwerveModuleState(
            StatusSignal<Double> position,
            StatusSignal<Double> velocity,
            StatusSignal<Double> acceleration,
            StatusSignal<Double> current,
            StatusSignal<Double> theta,
            StatusSignal<Double> omega)
    {
        previousState = new AdvanceSwerveModuleInstant();
        currentState = new AdvanceSwerveModuleInstant();
    }

    private class AdvanceSwerveModuleInstant {
        public double position;
        public double velocity;
        public double acceleration;
        public double current;
        public double torque;
        public double force;

        public double theta;
        public double omega;

        public double timestamp;

        public AdvanceSwerveModuleInstant() {
            position = 0;
            velocity = 0;
            acceleration = 0;
            current = 0;
            torque = 0;
            force = 0;
            theta = 0;
            omega = 0;
            timestamp = 0;
        }

        public AdvanceSwerveModuleInstant(
                StatusSignal<Double> position,
                StatusSignal<Double> velocity,
                StatusSignal<Double> acceleration,
                StatusSignal<Double> current,
                StatusSignal<Double> theta,
                StatusSignal<Double> omega)
        {
            this.timestamp = Utils.getCurrentTimeSeconds();
            calculatePosition(timestamp, position, velocity, acceleration);

            // Calculate force
            this.current = current.getValue();
            this.torque = DCMotor.getKrakenX60Foc(1).getTorque(this.current);
            this.force = Units.inchesToMeters(2); // NM to N
            this.force = this.force * DrivetrainConstants.DRIVE_GEAR_RATIO; // Accounting for the gear ratio

            //Theta
            this.theta = BaseStatusSignal.getLatencyCompensatedValue(theta, omega);
            this.omega = omega.getValue();
        }

        private void calculatePosition(
                double timestamp,
                StatusSignal<Double> position,
                StatusSignal<Double> velocity,
                StatusSignal<Double> acceleration
                ){

            // First step is to solve for velocity at the time when the acceleration is taken using v=vo+at
            double accelerationTimestamp = acceleration.getAllTimestamps().getCANivoreTimestamp().getTime();
            double velocityTimestamp = velocity.getAllTimestamps().getCANivoreTimestamp().getTime();
            double positionTimestamp = velocity.getAllTimestamps().getCANivoreTimestamp().getTime();

            double velocityAtAcceleration = velocity.getValue() + acceleration.getValue() * (accelerationTimestamp - velocityTimestamp);
            double xAccelerationTimeDif = accelerationTimestamp - positionTimestamp;
            double xAtAccelerationTime = position.getValue() + (velocityAtAcceleration * xAccelerationTimeDif) + (.5 * acceleration.getValue() * xAccelerationTimeDif);
            double finalTimestamp = timestamp - accelerationTimestamp;

            this.position = xAtAccelerationTime + (velocityAtAcceleration * finalTimestamp) + (.5 * acceleration.getValue() * finalTimestamp);
            this.position = this.position / DrivetrainConstants.DRIVE_GEAR_RATIO;
            this.position = this.position * (Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI); // Position is now in meters

            this.velocity = BaseStatusSignal.getLatencyCompensatedValue(velocity,acceleration);
            this.velocity = this.velocity / DrivetrainConstants.DRIVE_GEAR_RATIO;
            this.velocity = this.velocity * (Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI); // Position is now in meters

            this.acceleration = acceleration.getValue();
            this.acceleration = this.acceleration / DrivetrainConstants.DRIVE_GEAR_RATIO;
            this.acceleration = this.acceleration * (Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI); // Position is now in meters
        }
    }
}
