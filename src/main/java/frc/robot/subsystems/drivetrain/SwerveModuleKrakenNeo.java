package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleKrakenNeo implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveConfig;
    private final MotionMagicConfigs motionMagicConfigs;

    private final CANSparkMax steerMotor;
    private final CANcoder encoder;
    private final SwerveModule module;

    private SwerveModuleState targetState;
    private SwerveModuleState prevState;
    private SwerveModuleState currentState;
    private SwerveModulePosition modulePosition;

    public SwerveModuleKrakenNeo(int driveID, int steerID, int encoderID, SwerveModule module) {
        driveMotor = new TalonFX(driveID, "Drivetrain");
        driveConfig = new TalonFXConfiguration();

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = DrivetrainConstants.MAX_RPM_FOC;

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        Slot0Configs velocityControllerConfig = new Slot0Configs();
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        AudioConfigs audioConfig = new AudioConfigs();

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
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        audioConfig.BeepOnConfig = true;
        audioConfig.AllowMusicDurDisable = true;
        audioConfig.BeepOnBoot = true;

        driveConfig.CurrentLimits = currentLimitConfig;
        driveConfig.Voltage = voltageConfigs;
        driveConfig.Slot0 = velocityControllerConfig;
        driveConfig.Feedback = feedbackConfig;
        driveConfig.Audio = audioConfig;

        driveMotor.getConfigurator().apply(driveConfig);

        steerMotor = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);

        steerMotor.setSmartCurrentLimit(20);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerMotor.enableVoltageCompensation(12);
        this.module = module;

        encoder = new CANcoder(encoderID, "Drivetrain");

        targetState = new SwerveModuleState();
        prevState = new SwerveModuleState();
        currentState = new SwerveModuleState();
    }
    @Override
    public void readPeriodic() {
        currentState = new SwerveModuleState();
    }

    @Override
    public void writePeriodic() {
        MotionMagicVelocityVoltage driveControl = new MotionMagicVelocityVoltage(50);
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.prevState = this.targetState;
        this.targetState = targetState;
    }
    @Override
    public SwerveModuleState getTargetState() {
        return null;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return null;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return modulePosition;
    }
}
