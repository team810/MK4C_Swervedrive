package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleKrakenNeo implements SwerveModuleIO {
    // Drive Motor
    private final TalonFX driveMotor;
    private VelocityVoltage driveControl;

    private final StatusSignal<Double> drivePosition; // R
    private final StatusSignal<Double> driveVelocity; // R/S
    private final StatusSignal<Double> driveAcceleration; //R/S^2
    private final StatusSignal<Double> driveTorqueCurrent;

    private final StatusSignal<Double> driveCurrentDraw;
    private final StatusSignal<Double> driveTemperature;
    private final StatusSignal<Double> driveVoltage;

    // Steer Motor
    private final CANSparkMax steerMotor;
    private final PIDController steerController;

    // Encoder
    private final CANcoder encoder;

    private final StatusSignal<Double> steerAngle;
    private final StatusSignal<Double> steerVelocity;

    // General
    private SwerveModuleState currentState;
    private SwerveModuleState targetState;
    private final SwerveModule module;
    private final String moduleName;

    public SwerveModuleKrakenNeo(int driveID, int steerID, int encoderID, SwerveModule module)
    {
        // Drive motor configuration
        driveMotor = new TalonFX(driveID, DrivetrainConstants.CAN_BUS);
        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig(module));

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, drivePosition,driveVelocity,driveAcceleration,driveTorqueCurrent);

        driveCurrentDraw = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();
        driveVoltage = driveMotor.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(10, driveCurrentDraw,driveVelocity,driveAcceleration);

        driveMotor.optimizeBusUtilization();

        // Steer motor configuration
        steerMotor = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
        steerMotor.setSmartCurrentLimit(40);
        steerMotor.enableVoltageCompensation(12);
        steerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        steerMotor.clearFaults();

        steerController = new PIDController(0,0,0);
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        steerController.setTolerance(Units.degreesToRadians(1));

        // Encoder configuration
        encoder = new CANcoder(encoderID, DrivetrainConstants.CAN_BUS);
        encoder.getConfigurator().apply(DrivetrainConstants.getEncoderConfig());

        steerAngle = encoder.getAbsolutePosition();
        steerVelocity = encoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(100, steerAngle,steerVelocity);

        // General Configuration
        currentState = new SwerveModuleState();
        this.module = module;

        switch (this.module)
        {
            case FrontLeft -> {
                moduleName = "FrontLeft";
            }
            case FrontRight -> {
                moduleName = "FrontRight";
            }
            case BackLeft -> {
                moduleName = "BackLeft";
            }
            case BackRight -> {
                moduleName = "BackRight";
            }
            default -> {
                moduleName = "Unknown";
            }
        }
    }

    @Override
    public void readPeriodic() {
        double velocity = BaseStatusSignal.getLatencyCompensatedValue(driveVelocity,driveAcceleration);
        double acceleration = driveAcceleration.getValue();
        velocity = velocity / DrivetrainConstants.DRIVE_GEAR_RATIO;
        acceleration = acceleration / DrivetrainConstants.DRIVE_GEAR_RATIO;
        velocity = velocity * Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI;
        acceleration = acceleration * Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI;
        Rotation2d angle = Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(steerAngle,steerVelocity));

        currentState = new SwerveModuleState(
                velocity,
                angle
        );

        // General Logging
        Logger.recordOutput("Drivetrain/" + moduleName + "/CurrentState", currentState);
        Logger.recordOutput("Drivetrain/" + moduleName + "/TargetState", targetState);
        // Drive Motor Logging
        Logger.recordOutput("Drivetrain/" + moduleName + "Drivetrain/Position",getModulePosition(Utils.getCurrentTimeSeconds()));
        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Velocity", velocity);
        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Acceleration", acceleration);

        DCMotor.getKrakenX60Foc(1).getTorque(driveMotor.getTorqueCurrent().getValue());

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentState().angle);
    }
    @Override
    public SwerveModuleState getCurrentState() {
        return currentState;
    }



    @Override
    public SwerveModulePosition getModulePosition(double timestamp) {
        double theta = BaseStatusSignal.getLatencyCompensatedValue(steerAngle, steerVelocity);

        return new SwerveModulePosition(calculatePosition(timestamp),Rotation2d.fromDegrees(theta));
    }
}
