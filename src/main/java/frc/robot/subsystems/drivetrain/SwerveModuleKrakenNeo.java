package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModuleKrakenNeo implements SwerveModuleIO {
    private final TalonFX driveMotor;

    private final CANSparkMax steerMotor;
    private final PIDController steerController;
    private final CANcoder encoder;

    private final StatusSignal<Double> drivePositionSignal; // Drive motor position updated every 20 ms
    private final StatusSignal<Double> driveVelocitySignal; // Drive motor velocity updated ever 20 ms
    private final StatusSignal<Double> driveAccelerationSignal; // Drive motor acceleration updated every 20 ms
    private final StatusSignal<Double> wheelAngle; // -.5 to .5
    private final StatusSignal<Double> driveTempSignal; // Motor Temperature updated ever 100 ms
    private final StatusSignal<Double> driveVoltageSignal; // Applied Voltage updated every 100ms
    private final StatusSignal<Double> driveCurrentSignal; // Current draw updated every 100 ms

    private final SwerveModule module;

    private SwerveModuleState targetState;
    private SwerveModuleState prevState;
    private SwerveModuleState currentState;
    private SwerveModulePosition modulePosition;

    public SwerveModuleKrakenNeo(int driveID, int steerID, int encoderID, SwerveModule module) {
        driveMotor = new TalonFX(driveID, "Drivetrain");
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAccelerationSignal = driveMotor.getAcceleration();

        driveTempSignal = driveMotor.getDeviceTemp();
        driveVoltageSignal = driveMotor.getSupplyVoltage();
        driveCurrentSignal = driveMotor.getSupplyCurrent();

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
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

        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        audioConfig.BeepOnConfig = true;
        audioConfig.AllowMusicDurDisable = true;
        audioConfig.BeepOnBoot = true;

        driveConfig.CurrentLimits = currentLimitConfig;
        driveConfig.Voltage = voltageConfigs;
        driveConfig.Slot0 = velocityControllerConfig;
        driveConfig.Feedback = feedbackConfig;
        driveConfig.Audio = audioConfig;
        driveConfig.MotionMagic = motionMagicConfigs;

        driveMotor.getConfigurator().apply(driveConfig);

        steerMotor = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);

        steerMotor.setSmartCurrentLimit(20);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerMotor.enableVoltageCompensation(12);
        this.module = module;

        steerController = new PIDController(0,0,0); // Measurement and Setpoint are in radians -PI to PI.
        steerController.setP(0); // Kp = output/error
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        steerController.setTolerance((2 * Math.PI) * .01); // 1 percent of a ration error

        encoder = new CANcoder(encoderID, "Drivetrain");
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(config);
        wheelAngle = encoder.getAbsolutePosition();

        targetState = new SwerveModuleState();
        prevState = new SwerveModuleState();
        currentState = new SwerveModuleState();
    }
    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        double targetVelocity = (targetState.speedMetersPerSecond/Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES * Math.PI)) * DrivetrainConstants.DRIVE_GEAR_RATIO;
        targetVelocity = MathUtil.clamp(targetVelocity, -DrivetrainConstants.MAX_RPM_FOC, DrivetrainConstants.MAX_RPM_FOC);
        MotionMagicVelocityVoltage control = new MotionMagicVelocityVoltage(targetVelocity);
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.prevState = this.targetState;
        this.targetState = targetState;
    }
    @Override
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return modulePosition;
    }
}
