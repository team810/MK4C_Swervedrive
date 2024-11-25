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

    private ModuleObservationRaw frontLeftObservation;
    private final ReadWriteLock frontLeftObservationLock;
    private final ModuleSignals frontLeftSignals;

    private ModuleObservationRaw frontRightObservation;
    private final ReadWriteLock frontRightObservationLock;
    private final ModuleSignals frontRightSignals;

    private ModuleObservationRaw backLeftObservation;
    private final ReadWriteLock backLeftObservationLock;
    private final ModuleSignals backLeftSignals;
    
    private ModuleObservationRaw backRightObservation;
    private final ReadWriteLock backRightObservationLock;
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

        frontLeftObservationLock = new ReentrantReadWriteLock();
        frontLeftObservation = new ModuleObservationRaw();
        frontLeftSignals = frontLeft;

        frontRightObservationLock = new ReentrantReadWriteLock();
        frontRightObservation = new ModuleObservationRaw();
        frontRightSignals = frontRight;

        backLeftObservationLock = new ReentrantReadWriteLock();
        backLeftObservation = new ModuleObservationRaw();
        backLeftSignals = backLeft;

        backRightObservationLock = new ReentrantReadWriteLock();
        backRightObservation = new ModuleObservationRaw();
        backRightSignals = backRight;

        yawLock = new ReentrantReadWriteLock();
        yawSignal = yaw;

        BaseStatusSignal.setUpdateFrequencyForAll(
                500,
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
            frontLeftObservationLock.writeLock().lock();
            frontLeftObservation = SignalExtract(frontLeftSignals);
            frontLeftObservationLock.writeLock().unlock();

            frontRightObservationLock.writeLock().lock();
            frontRightObservation = SignalExtract(frontRightSignals);
            frontRightObservationLock.writeLock().unlock();

            backLeftObservationLock.writeLock().lock();
            backLeftObservation = SignalExtract(backLeftSignals);
            backLeftObservationLock.writeLock().unlock();

            backRightObservationLock.writeLock().lock();
            backRightObservation = SignalExtract(backRightSignals);
            backRightObservationLock.writeLock().unlock();

            observationsLock.writeLock().lock();
            frontLeftObservationLock.readLock().lock();
            frontRightObservationLock.readLock().lock();
            backLeftObservationLock.readLock().lock();
            backRightObservationLock.readLock().lock();
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
            frontLeftObservationLock.readLock().unlock();
            frontRightObservationLock.readLock().unlock();
            backLeftObservationLock.readLock().unlock();
            backRightObservationLock.readLock().unlock();
            yawLock.readLock().unlock();
        }
    }

    public ModuleObservationRaw SignalExtract(ModuleSignals signals)
    {
        ModuleObservationRaw raw = new ModuleObservationRaw();
        raw.position = signals.positionSignal.getValue();
        raw.velocity = signals.velocitySignal.getValue();
        raw.acceleration = signals.accelerationSignal.getValue();
        raw.current = signals.currentSignal.getValue();
        raw.appliedVoltage = signals.appliedVoltageSignal.getValue();
        raw.theta = signals.thetaSignal.getValue();
        raw.omega = signals.omegaSignal.getValue();
        return raw;
    }

    public ArrayList<SwerveObservation> getObservations()
    {

        observationsLock.readLock().lock();
        ArrayList<SwerveObservation> data = observations;
        observationsLock.readLock().unlock();

        return data;
    }

    public void clearObservations()
    {
        observationsLock.writeLock().lock();
        observations.clear();
        observationsLock.writeLock().unlock();
    }


    public ModuleObservationRaw getModuleObservations(SwerveModuleID id) {
        ModuleObservationRaw observation = new ModuleObservationRaw();
        switch (id)
        {
            case FrontLeft -> {
                frontLeftObservationLock.readLock().lock();
                observation = frontLeftObservation;
                frontLeftObservationLock.readLock().unlock();
            }
            case FrontRight -> {
                frontRightObservationLock.readLock().lock();
                observation = frontRightObservation;
                frontRightObservationLock.readLock().unlock();
            }
            case BackLeft -> {
                backLeftObservationLock.readLock().lock();
                observation = backLeftObservation;
                backLeftObservationLock.readLock().unlock();
            }
            case BackRight -> {
                backRightObservationLock.readLock().lock();
                observation = backRightObservation;
                backRightObservationLock.readLock().unlock();
            }
        }
        return observation;
    }

    public double getYaw() {
        yawLock.readLock().lock();
        double yaw = yawSignal.getValue();
        yawLock.readLock().unlock();
        return yaw;
    }
}
