package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private static DrivetrainSubsystem instance;

    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSimState;

    private final Observer observer;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;

    // Any speeds that come in are either going to be robot relative or fully filed relative
    private ChassisSpeeds velocityFOC;
    private ChassisSpeeds velocityRR;
    private final VelocityThetaControlFOC velocityThetaControlFOC;
    private ControlMethods controlMethod;

    private ChassisSpeeds targetSpeed;

    private Pose2d visionPose = new Pose2d();

    private DrivetrainSubsystem() {
        frontLeft = new KrakenNeoModule(SwerveModuleID.FrontLeft);
        frontRight = new KrakenNeoModule(SwerveModuleID.FrontRight);
        backLeft = new KrakenNeoModule(SwerveModuleID.BackLeft);
        backRight = new KrakenNeoModule(SwerveModuleID.BackRight);

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CAN_BUS);
        gyro.getConfigurator().apply(DrivetrainConstants.getGyroConfig());
        gyroSimState = gyro.getSimState();
        gyro.reset();

        observer = new Observer(
                frontLeft.getModuleSignals(),
                frontRight.getModuleSignals(),
                backLeft.getModuleSignals(),
                backRight.getModuleSignals(),
                gyro.getYaw()
        );
        observer.start();

        kinematics = DrivetrainConstants.getKinematics();

        Observer.SwerveObservation observation = observer.getObservations().get(0);
        observer.clearObservations();
        odometry = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[]{observation.frontLeft, observation.frontRight, observation.backLeft, observation.backRight},
                new Pose2d()
        );

        targetSpeed = new ChassisSpeeds(0,0,0);
        velocityFOC = new ChassisSpeeds(0,0,0);
        velocityRR = new ChassisSpeeds(0,0,0);
        velocityThetaControlFOC = new VelocityThetaControlFOC();
        controlMethod = ControlMethods.off;

        if (DrivetrainConstants.USING_VISION) {
            // Change the camera pose relative to robot center (x forward, y left, z up, degrees)

            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_NAME, Units.inchesToMeters(14.75), Units.inchesToMeters(4),Units.inchesToMeters(23),-4,25,0);
        }
    }


    @Override
    public void readPeriodic() {
        var moduleObservations = observer.getModuleObservations();
        frontLeft.readPeriodic(moduleObservations[0]);
        frontRight.readPeriodic(moduleObservations[1]);
        backLeft.readPeriodic(moduleObservations[2]);
        backRight.readPeriodic(moduleObservations[3]);

        if (Robot.isSimulation())
        {
            gyroSimState.addYaw(Units.radiansToDegrees(kinematics.toChassisSpeeds(frontLeft.getCurrentState(),frontRight.getCurrentState(),backLeft.getCurrentState(),backRight.getCurrentState()).omegaRadiansPerSecond * Robot.PERIOD));
        }

        if (Robot.isReal() && DrivetrainConstants.USING_VISION)
        {
            boolean reject = false;

            LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_NAME, odometry.getEstimatedPosition().getRotation().getDegrees(), gyro.getRate(),0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(DrivetrainConstants.LIME_LIGHT_NAME);

            if (mt2 != null)
            {
                if(Math.abs(Math.toRadians(gyro.getRate())) > DrivetrainConstants.MAX_ANGULAR_VELOCITY_ACCEPT_VISION_DATA)
                {
                    reject = true;
                }
                if(mt2.tagCount == 0)
                {
                    reject = true;
                }
                if(!reject)
                {
                    visionPose = mt2.pose;
                    odometry.addVisionMeasurement(visionPose, mt2.timestampSeconds);
                }
            }
        }

        Logger.recordOutput("VisionPose", visionPose);



        ArrayList<Observer.SwerveObservation> observations = observer.getObservations();
        for (int i = 0; i < observations.size(); i++) {
            odometry.updateWithTime(
                    observations.get(i).timestamp,
                    observations.get(i).yaw,
                    new SwerveModulePosition[]{
                            observations.get(i).frontLeft,
                            observations.get(i).frontRight,
                            observations.get(i).backLeft,
                            observations.get(i).backRight
                    });
        }
        observer.clearObservations();


        Logger.recordOutput("Drivetrain/Current/CurrentState", frontLeft.getCurrentState(), frontRight.getCurrentState(), backLeft.getCurrentState(), backRight.getCurrentState());
        Logger.recordOutput("Drivetrain/Current/CurrentPose", odometry.getEstimatedPosition());
    }

    @Override
    public void writePeriodic() {
        switch (controlMethod) {
            case off -> {
                targetSpeed = new ChassisSpeeds(0,0,0);
            }
            case VelocityFOC -> {
                targetSpeed = velocityFOC;
            }
            case VelocityRR -> {
                targetSpeed = velocityRR;
            }
            case VelocityThetaControlFOC -> {
                targetSpeed = velocityThetaControlFOC.getTargetSpeeds(odometry.getEstimatedPosition().getRotation());
            }
        }
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

    public Pose2d getPose()
    {
        return odometry.getEstimatedPosition();
    }

    public void resetGyro() {
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue)
        {
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
        }
    }

    public void switchAlliances() {
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue)
        {
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
        }
    }

    public void resetPose(Pose2d pose) {

        odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeft.getPosition(),Rotation2d.fromRadians(frontLeft.getTheta())),
                new SwerveModulePosition(frontRight.getPosition(),Rotation2d.fromRadians(frontRight.getTheta())),
                new SwerveModulePosition(backLeft.getPosition(),Rotation2d.fromRadians(backLeft.getTheta())),
                new SwerveModulePosition(backRight.getPosition(),Rotation2d.fromRadians(backRight.getTheta()))},
                pose);
    }

    public void setControlMode(ControlMethods control) {
        this.controlMethod = control;
    }


    public void setVelocityFOC(ChassisSpeeds targetSpeed) {
        this.velocityFOC = targetSpeed;
    }

    public void setVelocityThetaControlFOC(double horizontalSpeed, double verticalSpeed, Rotation2d targetAngle) {
        velocityThetaControlFOC.setControl(horizontalSpeed, verticalSpeed, targetAngle);
    }

    public ChassisSpeeds getVelocityRR() {
        return velocityRR;
    }

    public void setVelocityRR(ChassisSpeeds velocityRR) {
        this.velocityRR = velocityRR;
    }

    private static class VelocityThetaControlFOC {
        private final PIDController thetaController;
        private double horizontalSpeed = 0;
        private double verticalSpeed = 0;
        private Rotation2d targetAngle = new Rotation2d();
        private final SlewRateLimiter limiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

        public VelocityThetaControlFOC() {
            thetaController = new PIDController(
                    8,
                    0,
                    0
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public void setControl(double horizontalSpeed, double verticalSpeed, Rotation2d targetAngle) {
            this.horizontalSpeed = horizontalSpeed;
            this.verticalSpeed = verticalSpeed;
            this.targetAngle = targetAngle;
        }
        public ChassisSpeeds getTargetSpeeds(Rotation2d currentAngle) {
            double omega = thetaController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
            omega = MathUtil.clamp(omega, -10,10);
            limiter.calculate(omega);
            return new ChassisSpeeds(horizontalSpeed, verticalSpeed, omega);
        }
    }

    public enum ControlMethods {
        off,
        VelocityFOC, // Velocity control filed relative
        VelocityRR, // Velocity control robot relative
        VelocityThetaControlFOC,
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }
}
