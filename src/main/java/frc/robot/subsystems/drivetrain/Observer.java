package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class Observer extends Thread {
    private final ArrayList<SwerveObservation> observations;
    private final ReadWriteLock observationsLock;

    private final ReadWriteLock moduleObservationLock;

    private ModuleObservationRaw frontLeftObservation;
    private final ModuleSignals frontLeftSignals;

    private ModuleObservationRaw frontRightObservation;
    private final ModuleSignals frontRightSignals;

    private ModuleObservationRaw backLeftObservation;
    private final ModuleSignals backLeftSignals;

    private ModuleObservationRaw backRightObservation;
    private final ModuleSignals backRightSignals;

    private final ReadWriteLock yawLock;
    private final StatusSignal<Double> yawSignal;

    public static class SwerveObservation {
        SwerveModulePosition frontLeft;
        SwerveModulePosition frontRight;
        SwerveModulePosition backLeft;
        SwerveModulePosition backRight;

        Rotation2d yaw;

        double timestamp;

        public SwerveObservation(SwerveModulePosition frontLeft, SwerveModulePosition frontRight, SwerveModulePosition backLeft, SwerveModulePosition backRight, Rotation2d yaw, double timestamp) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
            this.yaw = yaw;
            this.timestamp = timestamp;
        }
    }

    public static class ModuleObservationRaw {
        public double position;
        public double velocity;
        public double acceleration;
        public double current;
        public double appliedVoltage;
        public double theta;
        public double omega;

        public ModuleObservationRaw() {}
    }

    public static class ModuleSignals
    {
        public StatusSignal<Double> positionSignal;
        public StatusSignal<Double> velocitySignal;
        public StatusSignal<Double> accelerationSignal;
        public StatusSignal<Double> currentSignal;
        public StatusSignal<Double> appliedVoltageSignal;
        public StatusSignal<Double> thetaSignal;
        public StatusSignal<Double> omegaSignal;

        public ModuleSignals(StatusSignal<Double> positionSignal, StatusSignal<Double> velocitySignal, StatusSignal<Double> accelerationSignal, StatusSignal<Double> currentSignal, StatusSignal<Double> appliedVoltageSignal, StatusSignal<Double> thetaSignal, StatusSignal<Double> omegaSignal) {
            this.positionSignal = positionSignal;
            this.velocitySignal = velocitySignal;
            this.accelerationSignal = accelerationSignal;
            this.currentSignal = currentSignal;
            this.appliedVoltageSignal = appliedVoltageSignal;
            this.thetaSignal = thetaSignal;
            this.omegaSignal = omegaSignal;
        }

        public ModuleSignals() {}
    }

    public Observer(ModuleSignals frontLeft, ModuleSignals frontRight, ModuleSignals backLeft, ModuleSignals backRight, StatusSignal<Double> yaw) {
        observationsLock = new ReentrantReadWriteLock();
        observations = new ArrayList<>();
        observationsLock.writeLock().lock();
        observations.add(new SwerveObservation(new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new Rotation2d(0),0));
        observationsLock.writeLock().unlock();

        moduleObservationLock = new ReentrantReadWriteLock();

        frontLeftObservation = new ModuleObservationRaw();
        frontLeftSignals = frontLeft;

        frontRightObservation = new ModuleObservationRaw();
        frontRightSignals = frontRight;

        backLeftObservation = new ModuleObservationRaw();
        backLeftSignals = backLeft;

        backRightObservation = new ModuleObservationRaw();
        backRightSignals = backRight;

        yawLock = new ReentrantReadWriteLock();
        yawSignal = yaw;
        // The can bus is at 50% usage at 500 hz, should be able to go up to 800
        BaseStatusSignal.setUpdateFrequencyForAll(
                400,
                frontLeftSignals.positionSignal,
                frontLeftSignals.velocitySignal,
                frontLeftSignals.accelerationSignal,
                frontLeftSignals.currentSignal,
                frontLeftSignals.appliedVoltageSignal,
                frontLeftSignals.thetaSignal,
                frontLeftSignals.omegaSignal,

                frontRightSignals.positionSignal,
                frontRightSignals.velocitySignal,
                frontRightSignals.accelerationSignal,
                frontRightSignals.currentSignal,
                frontRightSignals.appliedVoltageSignal,
                frontRightSignals.thetaSignal,
                frontRightSignals.omegaSignal,

                backLeftSignals.positionSignal,
                backLeftSignals.velocitySignal,
                backLeftSignals.accelerationSignal,
                backLeftSignals.currentSignal,
                backLeftSignals.appliedVoltageSignal,
                backLeftSignals.thetaSignal,
                backLeftSignals.omegaSignal,

                backRightSignals.positionSignal,
                backRightSignals.velocitySignal,
                backRightSignals.accelerationSignal,
                backRightSignals.currentSignal,
                backRightSignals.thetaSignal,
                backRightSignals.omegaSignal,

                yawSignal
        );
    }

    @Override
    public void run() {
        while (true) {
            BaseStatusSignal.waitForAll(
                    1,
                    frontLeftSignals.positionSignal,
                    frontLeftSignals.velocitySignal,
                    frontLeftSignals.accelerationSignal,
                    frontLeftSignals.currentSignal,
                    frontLeftSignals.appliedVoltageSignal,
                    frontLeftSignals.thetaSignal,
                    frontLeftSignals.omegaSignal,

                    frontRightSignals.positionSignal,
                    frontRightSignals.velocitySignal,
                    frontRightSignals.accelerationSignal,
                    frontRightSignals.currentSignal,
                    frontRightSignals.appliedVoltageSignal,
                    frontRightSignals.thetaSignal,
                    frontRightSignals.omegaSignal,

                    backLeftSignals.positionSignal,
                    backLeftSignals.velocitySignal,
                    backLeftSignals.accelerationSignal,
                    backLeftSignals.currentSignal,
                    backLeftSignals.appliedVoltageSignal,
                    backLeftSignals.thetaSignal,
                    backLeftSignals.omegaSignal,

                    backRightSignals.positionSignal,
                    backRightSignals.velocitySignal,
                    backRightSignals.accelerationSignal,
                    backRightSignals.currentSignal,
                    backRightSignals.thetaSignal,
                    backRightSignals.omegaSignal,

                    yawSignal
            );
            moduleObservationLock.writeLock().lock();

            frontLeftObservation = SignalExtract(frontLeftSignals);
            frontRightObservation = SignalExtract(frontRightSignals);
            backLeftObservation = SignalExtract(backLeftSignals);
            backRightObservation = SignalExtract(backRightSignals);

            moduleObservationLock.writeLock().unlock();

            observationsLock.writeLock().lock();
            moduleObservationLock.readLock().lock();
            yawLock.readLock().lock();

            observations.add(
                    new SwerveObservation(
                            new SwerveModulePosition((frontLeftObservation.position/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(frontLeftObservation.theta)),
                            new SwerveModulePosition((frontRightObservation.position/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(frontRightObservation.theta)),
                            new SwerveModulePosition((backLeftObservation.position/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(backLeftObservation.theta)),
                            new SwerveModulePosition((backRightObservation.position/DrivetrainConstants.DRIVE_GEAR_RATIO) * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI, Rotation2d.fromRotations(backRightObservation.theta)),
                            Rotation2d.fromDegrees(getYaw()),
                            Timer.getFPGATimestamp()
                    )
            );

            observationsLock.writeLock().unlock();
            moduleObservationLock.readLock().unlock();
            yawLock.readLock().unlock();
        }
    }

    public ModuleObservationRaw SignalExtract(ModuleSignals signals) {
        ModuleObservationRaw raw = new ModuleObservationRaw();
        raw.position = StatusSignal.getLatencyCompensatedValue(signals.positionSignal,signals.velocitySignal);
        raw.velocity = StatusSignal.getLatencyCompensatedValue(signals.velocitySignal,signals.accelerationSignal);
        raw.acceleration = signals.accelerationSignal.getValue();
        raw.current = signals.currentSignal.getValue();
        raw.appliedVoltage = signals.appliedVoltageSignal.getValue();
        raw.theta = StatusSignal.getLatencyCompensatedValue(signals.thetaSignal, signals.omegaSignal);
        raw.omega = signals.omegaSignal.getValue();
        return raw;
    }

    public ArrayList<SwerveObservation> getObservations() {
        observationsLock.readLock().lock();
        ArrayList<SwerveObservation> data = observations;
        observationsLock.readLock().unlock();

        return data;
    }

    public void clearObservations() {
        observationsLock.writeLock().lock();
        observations.clear();
        observationsLock.writeLock().unlock();
    }


    public ModuleObservationRaw[] getModuleObservations() {
        moduleObservationLock.readLock().lock();
        var moduleObservations =  new ModuleObservationRaw[]{frontLeftObservation, frontRightObservation, backLeftObservation, backRightObservation};
        moduleObservationLock.readLock().unlock();
        return moduleObservations;
    }

    public double getYaw() {
        yawLock.readLock().lock();
        double yaw = yawSignal.getValue();
        yawLock.readLock().unlock();
        return yaw;
    }
}
