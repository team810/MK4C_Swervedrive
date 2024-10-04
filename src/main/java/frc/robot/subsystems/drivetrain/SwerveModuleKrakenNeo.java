package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleKrakenNeo implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveConfig;
    private final MotionMagicConfigs motionMagicConfigs;


    private final CANSparkMax steerMotor;
    private final CANcoder encoder;
    private final SwerveModule module;

    public SwerveModuleKrakenNeo(int driveID, int steerID, int encoderID, SwerveModule module) {
        driveMotor = new TalonFX(driveID);
        driveConfig = new TalonFXConfiguration();

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity =

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        Slot0Configs velocityControllerConfig = new Slot0Configs();
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();

        currentLimitConfig.StatorCurrentLimitEnable = true;
        currentLimitConfig.SupplyCurrentLimitEnable = true;
        currentLimitConfig.StatorCurrentLimit = 10;

        voltageConfigs.PeakForwardVoltage = 12;
        voltageConfigs.PeakReverseVoltage = -12;

        velocityControllerConfig.kP = 0;
        velocityControllerConfig.kI = 0;
        velocityControllerConfig.kD = 0;
        velocityControllerConfig.kV = 0;
        velocityControllerConfig.kA = 0;
        velocityControllerConfig.kG = 0;

        feedbackConfig.SensorToMechanismRatio = DrivetrainConstants.DRIVE_GEAR_RATIO;

        driveConfig.withCurrentLimits(currentLimitConfig);
        driveConfig.withVoltage(voltageConfigs);
        driveConfig.withSlot0(velocityControllerConfig);
        driveConfig.withFeedback(feedbackConfig);

        driveMotor.getConfigurator().apply(driveConfig);

        steerMotor = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);

        steerMotor.setSmartCurrentLimit(20);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerMotor.enableVoltageCompensation(12);
        this.module = module;

        encoder = new CANcoder(encoderID);
    }
    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {

    }

    @Override
    public SwerveModuleState getTargetState() {
        return null;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return null;
    }
}
