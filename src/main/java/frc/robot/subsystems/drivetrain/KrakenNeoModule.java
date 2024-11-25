package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

// Current does not require pro
public class KrakenNeoModule implements SwerveModuleIO{
    private final SwerveModuleID id;
    private final String idString;

    private final TalonFX driveMotor;
    private final TalonFXSimState driveSimState;
    private final DCMotorSim driveMotorSim;
    private double driveAppliedVoltage;
    private VelocityVoltage driveMotorControl;

    private final CANSparkMax steerMotor;
    private final PIDController steerController;
    private final CANcoder encoder;
    private final CANcoderSimState encoderSimState;
    private double steerAppliedVoltage;

    private double position;
    private double velocity;
    private double acceleration;
    private double torque;
    private double force;

    private double theta;
    private double omega;

    private Observer.ModuleObservationRaw rawInput;

    private SwerveModuleState targetState;

    public KrakenNeoModule(SwerveModuleID id)
    {
        this.id = id;
        this.idString = this.id.toString();

        driveMotor = new TalonFX(DrivetrainConstants.getDriveID(id),DrivetrainConstants.CAN_BUS);
        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig(id));
        driveSimState = driveMotor.getSimState();
        driveMotorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1),1,DrivetrainConstants.MOMENT_OF_INTRA_DRIVE);

        steerMotor = new CANSparkMax(DrivetrainConstants.getSteerID(id), CANSparkLowLevel.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.clearFaults();
        steerMotor.setSmartCurrentLimit(20); // The steer motor does not really need that much torque, and it is completely fine to run these motors at 20 amps,
        steerMotor.enableVoltageCompensation(12);
        steerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        steerAppliedVoltage = 0;

        // We are not relying on really any feedback from the motors so we can set the periodic frames very high to reduces unnecessary canbus usage
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0,50);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1,100);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2,1000);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3,1000);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4,1000);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,1000);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6,1000);

        encoder = new CANcoder(DrivetrainConstants.getEncoderID(id), DrivetrainConstants.CAN_BUS);
        encoderSimState = encoder.getSimState();

        steerController = new PIDController(
                DrivetrainConstants.STEER_KP,
                DrivetrainConstants.STEER_KI,
                DrivetrainConstants.STEER_KD
        );
        steerController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.optimizeBusUtilization();
        encoder.optimizeBusUtilization();

        rawInput = new Observer.ModuleObservationRaw();
    }

    @Override
    public void readPeriodic(Observer.ModuleObservationRaw data) {
        rawInput = data;
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Position", getPosition());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Velocity", getVelocity());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Acceleration", getAcceleration());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedDriveVoltage", driveAppliedVoltage);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "DriveCurrent", rawInput.current);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Force", getForce());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Torque", getTorque());

        Logger.recordOutput("Drivetrain/" + idString + "/" + "Theta", getTheta());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Omega", getOmega());

        targetState = new SwerveModuleState();
    }

    @Override
    public void writePeriodic() {
        // Control over drive motor
        double velocity = targetState.speedMetersPerSecond;
        double acceleration = 0;
        velocity = (velocity/(DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI)) * DrivetrainConstants.DRIVE_GEAR_RATIO; // converts mps to rotations of motor per second
        acceleration = (velocity - ((getVelocity()/(DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI)) * DrivetrainConstants.DRIVE_GEAR_RATIO)) / Robot.PERIOD;

        driveMotorControl = new VelocityVoltage(velocity,acceleration,true,0,0,false,false,false);
        driveMotor.setControl(driveMotorControl);

        // Control over steer motor
        double targetAngle = MathUtil.angleModulus(targetState.angle.getRadians()); // The angle is rapped from -PI to PI
        steerAppliedVoltage = steerController.calculate(getTheta(),targetAngle);
        steerMotor.setVoltage(steerAppliedVoltage);

        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedState", targetState);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedVelocityRPM", velocity);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Target Angle", targetAngle);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedSteerVoltage", steerAppliedVoltage);
    }

    @Override
    public void moduleSim() {
        // Just drive motor

        driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
        driveMotorSim.update(Robot.PERIOD);

        driveSimState.setSupplyVoltage(12);

        driveSimState.setRawRotorPosition(driveMotorSim.getAngularPositionRotations()); // This is just in rotations
        driveSimState.setRotorVelocity(driveMotorSim.getAngularVelocityRPM()/60); // Converts RPM to RPS

        // Steer Simulation
        double simOmega = ((steerAppliedVoltage / 12) *  (DrivetrainConstants.MAX_RPM_FOC / 60)) / DrivetrainConstants.STEER_GEAR_RATIO; // rotations per second
        encoderSimState.setSupplyVoltage(12);
        encoderSimState.setVelocity(simOmega);
        encoderSimState.addPosition(simOmega * Robot.PERIOD);
    }


    @Override
    public synchronized double getPosition() {
        position = rawInput.position;
        position = position / DrivetrainConstants.DRIVE_GEAR_RATIO;
        position = position * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return position;
    }


    @Override
    public double getVelocity() {
        velocity = rawInput.velocity;
        velocity = velocity / DrivetrainConstants.DRIVE_GEAR_RATIO;
        velocity = velocity * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return velocity;
    }

    @Override
    public double getAcceleration() {
        acceleration = rawInput.acceleration;
        acceleration = acceleration / DrivetrainConstants.DRIVE_GEAR_RATIO;
        acceleration = acceleration * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return acceleration;
    }

    @Override
    public double getTorque() {
        torque = DCMotor.getKrakenX60Foc(1).getTorque(rawInput.current);
        torque = torque * DrivetrainConstants.DRIVE_GEAR_RATIO;
        return torque;
    }

    @Override
    public double getForce() {
        force = getTorque() * Units.inchesToMeters(2);
        return force;
    }

    @Override
    public double getDriveAppliedVoltage() {
        driveAppliedVoltage = rawInput.appliedVoltage;
        return driveAppliedVoltage;
    }

    @Override
    public synchronized double getTheta() {
        theta = rawInput.theta;
        theta = theta * 2 * Math.PI; // To convert from rotations to radians
        return theta;
    }

    @Override
    public double getOmega() {
        omega = rawInput.omega;
        omega = omega * 2 * Math.PI; // convert from rotations to radians
        return omega;
    }

    @Override
    public double getSteerAppliedVoltage() {
        return steerAppliedVoltage;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getTheta()));
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
    }

    @Override
    public Observer.ModuleSignals getModuleSignals() {
        return new Observer.ModuleSignals(
                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getAcceleration(),
                driveMotor.getTorqueCurrent(),
                driveMotor.getMotorVoltage(),
                encoder.getAbsolutePosition(),
                encoder.getVelocity()
        );
    }
}
