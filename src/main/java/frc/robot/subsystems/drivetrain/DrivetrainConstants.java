package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public final static String CAN_BUS = "drivetrain";

    public final static int FRONT_LEFT_DRIVE_ID = 1;
    public final static int FRONT_LEFT_STEER_ID = 2;
    public final static int FRONT_LEFT_ENCODER_ID = 9;

    public final static int FRONT_RIGHT_DRIVE_ID = 3;
    public final static int FRONT_RIGHT_STEER_ID = 4;
    public final static int FRONT_RIGHT_ENCODER_ID = 10;

    public final static int BACK_LEFT_DRIVE_ID = 5;
    public final static int BACK_LEFT_STEER_ID = 6;
    public final static int BACK_LEFT_ENCODER_ID = 11;

    public final static int BACK_RIGHT_DRIVE_ID = 7;
    public final static int BACK_RIGHT_STEER_ID = 8;
    public final static int BACK_RIGHT_ENCODER_ID = 12;

    public final static int GYRO_ID = 13;

    public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(28); // measure of FL wheel to FR wheel or BL wheel to BR wheel
    public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(28); // measure of FL wheel to BL wheel or FR wheel to BR wheel
    public static final double MAX_RPM_FOC = 5800;
    public static final double DRIVE_GEAR_RATIO = 5.36;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double MAX_VELOCITY = (MAX_RPM_FOC/60) * Units.inchesToMeters(WHEEL_DIAMETER_INCHES * Math.PI); // Meters per second
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(WHEEL_BASE_WIDTH / 2.0, WHEEL_BASE_LENGTH / 2.0);
    public static final double MAX_ACCELERATION = 8;
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(WHEEL_BASE_WIDTH / 2.0, WHEEL_BASE_LENGTH / 2.0);


    public static TalonFXConfiguration getDriveConfig(SwerveModule module) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimitEnable = true;
        currentLimitConfig.SupplyCurrentLimitEnable = true;
        currentLimitConfig.StatorCurrentLimit = 120;
        currentLimitConfig.SupplyCurrentLimit = 70;
        config.CurrentLimits = currentLimitConfig;

        VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12;
        voltageConfigs.PeakReverseVoltage = -12;
        config.Voltage = voltageConfigs;

        Slot0Configs velocityControllerConfig = new Slot0Configs();
        switch (module)
        {
            case FrontLeft -> {
                velocityControllerConfig.kP = 0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
                velocityControllerConfig.kV = 0.1224;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
            }
            case FrontRight -> {
                velocityControllerConfig.kP = 0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
                velocityControllerConfig.kV = 0.1224;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
            }
            case BackLeft -> {
                velocityControllerConfig.kP = 0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
                velocityControllerConfig.kV = 0.1224;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
            }
            case BackRight -> {
                velocityControllerConfig.kP = 0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
                velocityControllerConfig.kV = 0.1224;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
            }
        }
        config.Slot0 = velocityControllerConfig;

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback = feedbackConfig;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = DrivetrainConstants.MAX_RPM_FOC;
        config.MotionMagic = motionMagicConfigs;

        AudioConfigs audioConfig = new AudioConfigs();
        audioConfig.BeepOnConfig = true;
        audioConfig.AllowMusicDurDisable = true;
        audioConfig.BeepOnBoot = true;
        config.Audio = audioConfig;

        return config;
    }

    public static CANcoderConfiguration getEncoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        config.MagnetSensor = magnetSensorConfigs;

        config.FutureProofConfigs = true;

        return config;
    }
}
