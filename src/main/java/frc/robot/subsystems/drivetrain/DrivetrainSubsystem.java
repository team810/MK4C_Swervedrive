package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.AdvancedSubsystem;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private final Pigeon2 gyro;
    // Always blue cord system
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    private final SwerveModuleState frontLeftTargetState;
    private final SwerveModuleState frontRightTargetState;
    private final SwerveModuleState backLeftTargetState;
    private final SwerveModuleState backRightTargetState;

    private final SwerveModuleState frontLeftCurrentState;
    private final SwerveModuleState frontRightCurrentState;
    private final SwerveModuleState backLeftCurrentState;
    private final SwerveModuleState backRightCurrentState;

    private final SwerveModulePosition frontLeftPosition;
    private final SwerveModulePosition frontRightPosition;
    private final SwerveModulePosition backLeftPosition;
    private final SwerveModulePosition backRightPosition;

    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    private ChassisSpeeds currentChassisSpeeds; // This is the chassis speeds that are currently applied to the robots drivetrain
    private ChassisSpeeds teleopChassisSpeeds; // This is the chassis speeds from gotten directly from joystick input.

    public DrivetrainSubsystem() {
        kinematics = new SwerveDriveKinematics(
            new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                    -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0)
        );

        frontLeftPosition = new SwerveModulePosition();
        frontRightPosition = new SwerveModulePosition();
        backLeftPosition = new SwerveModulePosition();
        backRightPosition = new SwerveModulePosition();

        frontLeft = new SwerveModuleKrakenNeo(
                DrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                DrivetrainConstants.FRONT_LEFT_STEER_ID,
                DrivetrainConstants.FRONT_LEFT_ENCODER_ID,
                SwerveModule.FrontLeft
        );
        frontRight = new SwerveModuleKrakenNeo(
                DrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                DrivetrainConstants.FRONT_RIGHT_STEER_ID,
                DrivetrainConstants.FRONT_RIGHT_ENCODER_ID,
                SwerveModule.FrontRight
        );
        backLeft = new SwerveModuleKrakenNeo(
                DrivetrainConstants.BACK_LEFT_DRIVE_ID,
                DrivetrainConstants.BACK_LEFT_STEER_ID,
                DrivetrainConstants.BACK_LEFT_ENCODER_ID,
                SwerveModule.BackLeft
        );
        backRight = new SwerveModuleKrakenNeo(
                DrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                DrivetrainConstants.BACK_RIGHT_STEER_ID,
                DrivetrainConstants.BACK_RIGHT_ENCODER_ID,
                SwerveModule.BackRight
        );

        frontLeftCurrentState = frontLeft.getCurrentState();
        frontRightCurrentState = frontRight.getCurrentState();
        backLeftCurrentState = backLeft.getCurrentState();
        backRightCurrentState = backRight.getCurrentState();

        frontLeftTargetState = new SwerveModuleState();
        frontRightTargetState = new SwerveModuleState();
        backLeftTargetState = new SwerveModuleState();
        backRightTargetState = new SwerveModuleState();

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CAN_BUS);
        gyro.reset();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition},new Pose2d());

        currentChassisSpeeds = new ChassisSpeeds();
        teleopChassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void readPeriodic() {
        

    }

    @Override
    public void writePeriodic() {

    }
}
