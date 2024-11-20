package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private static DrivetrainSubsystem instance;

    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSimState;
    private final StatusSignal<Double> thetaSignal;

    private SwerveModulePosition frontLeftPosition;
    private SwerveModulePosition frontRightPosition;
    private SwerveModulePosition backLeftPosition;
    private SwerveModulePosition backRightPosition;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;

    // Any speeds that come in are either going to be robot relative or fully filed relative
    private ChassisSpeeds targetSpeed;

    private DrivetrainSubsystem() {
        frontLeft = new KrakenNeoModule(SwerveModuleID.FrontLeft);
        frontRight = new KrakenNeoModule(SwerveModuleID.FrontRight);
        backLeft = new KrakenNeoModule(SwerveModuleID.BackLeft);
        backRight = new KrakenNeoModule(SwerveModuleID.BackRight);

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CAN_BUS);
        gyro.getConfigurator().apply(DrivetrainConstants.getGyroConfig());
        gyroSimState = gyro.getSimState();
        thetaSignal = gyro.getYaw();
        gyro.reset();
        thetaSignal.setUpdateFrequency(100);

        frontLeftPosition = new SwerveModulePosition();
        frontRightPosition = new SwerveModulePosition();
        backLeftPosition = new SwerveModulePosition();
        backRightPosition = new SwerveModulePosition();
        kinematics = DrivetrainConstants.getKinematics();
        odometry = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition},
                new Pose2d()
        );

        targetSpeed = new ChassisSpeeds(0,0,0);

    }

    @Override
    public void readPeriodic() {
        BaseStatusSignal.refreshAll(thetaSignal);
        frontLeft.readPeriodic();
        frontRight.readPeriodic();
        backLeft.readPeriodic();
        backRight.readPeriodic();

        updateModulePositions();

        odometry.update(Rotation2d.fromDegrees(thetaSignal.getValue()), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition});

        Logger.recordOutput("Drivetrain/Current/CurrentState", frontLeft.getCurrentState(), frontRight.getCurrentState(), backLeft.getCurrentState(), backRight.getCurrentState());
        Logger.recordOutput("Drivetrain/Current/CurrentPose", odometry.getEstimatedPosition());
    }

    @Override
    public void writePeriodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeed);
        states[0] = SwerveModuleState.optimize(states[0], new Rotation2d(frontLeft.getTheta()));
        states[1] = SwerveModuleState.optimize(states[1], new Rotation2d(frontRight.getTheta()));
        states[2] = SwerveModuleState.optimize(states[2], new Rotation2d(backLeft.getTheta()));
        states[3] = SwerveModuleState.optimize(states[3], new Rotation2d(backRight.getTheta()));

        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);

        frontLeft.writePeriodic();
        frontRight.writePeriodic();
        backLeft.writePeriodic();
        backRight.writePeriodic();

        Logger.recordOutput("Drivetrain/Applied/Speeds",targetSpeed);
        Logger.recordOutput("Drivetrain/Applied/States",states);
    }



    @Override
    public void simulationPeriodic() {
        frontLeft.moduleSim();
        frontRight.moduleSim();
        backLeft.moduleSim();
        backRight.moduleSim();
    }

    public void setTargetSpeed(ChassisSpeeds speed) {
        this.targetSpeed = speed;
    }
    public Pose2d getPose()
    {
        return odometry.getEstimatedPosition();
    }

    public void resetGyro() {
        if (DriverStation.getAlliance().isPresent())
        {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            {
                resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
            }else{
                resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
            }
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }
    }

    public void switchAlliances() {
        if (DriverStation.getAlliance().isPresent())
        {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            {
                resetPose(new Pose2d(getPose().getX(), getPose().getY(), gyro.getRotation2d().minus(Rotation2d.fromRadians(Math.PI))));
            }else{
                resetPose(new Pose2d(getPose().getX(), getPose().getY(), gyro.getRotation2d()));
            }
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), gyro.getRotation2d()));
        }
    }
    public void resetPose(Pose2d pose) {
        updateModulePositions();

        odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition}, pose);
    }

    private void updateModulePositions() {
        double currentTime = Utils.getCurrentTimeSeconds();

        frontLeftPosition = new SwerveModulePosition(frontLeft.getPosition(currentTime),new Rotation2d(frontLeft.getTheta()));
        frontRightPosition = new SwerveModulePosition(frontRight.getPosition(currentTime),new Rotation2d(frontRight.getTheta()));
        backLeftPosition = new SwerveModulePosition(backLeft.getPosition(currentTime),new Rotation2d(backLeft.getTheta()));
        backRightPosition = new SwerveModulePosition(backRight.getPosition(currentTime), new Rotation2d(backRight.getTheta()));
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }
}
