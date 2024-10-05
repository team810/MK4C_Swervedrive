package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

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

    public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(28); // measure of FL wheel to FR wheel or BL wheel to BR wheel
    public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(28); // measure of FL wheel to BL wheel or FR wheel to BR wheel
    public static final double MAX_RPM_FOC = 5800;
    public static final double DRIVE_GEAR_RATIO = 5.36;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double MAX_VELOCITY = (MAX_RPM_FOC/60) * Units.inchesToMeters(WHEEL_DIAMETER_INCHES * Math.PI); // Meters per second
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(WHEEL_BASE_WIDTH / 2.0, WHEEL_BASE_LENGTH / 2.0);
    public static final double MAX_ACCELERATION = 8;
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(WHEEL_BASE_WIDTH / 2.0, WHEEL_BASE_LENGTH / 2.0);

}
