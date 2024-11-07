package frc.robot.subsystems.drivetrain;

public class SwerveModuleKrakenNeo {
//    // Drive Motor
//    private final TalonFX driveMotor;
//    private VelocityVoltage driveControl;
//
//    private final StatusSignal<Double> drivePosition; // R
//    private final StatusSignal<Double> driveVelocity; // R/S
//    private final StatusSignal<Double> driveAcceleration; //R/S^2
//    private final StatusSignal<Double> driveTorqueCurrent;
//
//    private final StatusSignal<Double> driveCurrentDraw;
//    private final StatusSignal<Double> driveTemperature;
//    private final StatusSignal<Double> driveVoltage;
//
//    // Steer Motor
//    private final CANSparkMax steerMotor;
//    private final PIDController steerController;
//
//    // Encoder
//    private final CANcoder encoder;
//
//    private final StatusSignal<Double> theta;
//    private final StatusSignal<Double> omega;
//
//    // Simulated Stuff
//    private final DCMotorSim driveMotorSim;
//    private final DCMotorSim steerMotorSim;
//
//    // General
//    private AdvancedSwerveModuleState currentState;
//    private SwerveModuleState targetState;
//    private SwerveModuleState prevTargetState;
//    private final SwerveModule module;
//    private final String moduleName;
//
//    public SwerveModuleKrakenNeo(int driveID, int steerID, int encoderID, SwerveModule module)
//    {
//        // Drive motor configuration
//        driveMotor = new TalonFX(driveID, DrivetrainConstants.CAN_BUS);
//        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig(module));
//
//        drivePosition = driveMotor.getPosition();
//        driveVelocity = driveMotor.getVelocity();
//        driveAcceleration = driveMotor.getAcceleration();
//        driveTorqueCurrent = driveMotor.getTorqueCurrent();
//
//        BaseStatusSignal.setUpdateFrequencyForAll(100, drivePosition,driveVelocity,driveAcceleration,driveTorqueCurrent);
//
//        driveCurrentDraw = driveMotor.getSupplyCurrent();
//        driveTemperature = driveMotor.getDeviceTemp();
//        driveVoltage = driveMotor.getSupplyVoltage();
//
//        BaseStatusSignal.setUpdateFrequencyForAll(10, driveCurrentDraw,driveVelocity,driveAcceleration);
//
//        driveMotor.optimizeBusUtilization();
//
//        // Steer motor configuration
//        steerMotor = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
//        steerMotor.setSmartCurrentLimit(40);
//        steerMotor.enableVoltageCompensation(12);
//        steerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
//        steerMotor.clearFaults();
//
//        steerController = new PIDController(0,0,0);
//        steerController.enableContinuousInput(-Math.PI, Math.PI);
//        steerController.setTolerance(Units.degreesToRadians(1));
//
//        // Encoder configuration
//        encoder = new CANcoder(encoderID, DrivetrainConstants.CAN_BUS);
//        encoder.getConfigurator().apply(DrivetrainConstants.getEncoderConfig());
//
//        theta = encoder.getAbsolutePosition();
//        omega = encoder.getVelocity();
//
//        BaseStatusSignal.setUpdateFrequencyForAll(100, theta,omega);
//
//        // Simulated Init
//        driveMotorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1),1,0.025/ DrivetrainConstants.DRIVE_GEAR_RATIO);
//        steerMotorSim = new DCMotorSim(DCMotor.getNEO(1),12.8,0.004096955);
//
//        // General Configuration
//        currentState = new AdvancedSwerveModuleState(
//                drivePosition,
//                driveVelocity,
//                driveAcceleration,
//                driveTorqueCurrent,
//                theta,
//                omega
//        );
//
//        this.module = module;
//        switch (this.module)
//        {
//            case FrontLeft -> {
//                moduleName = "FrontLeft";
//            }
//            case FrontRight -> {
//                moduleName = "FrontRight";
//            }
//            case BackLeft -> {
//                moduleName = "BackLeft";
//            }
//            case BackRight -> {
//                moduleName = "BackRight";
//            }
//            default -> {
//                moduleName = "Unknown";
//            }
//        }
//
//        prevTargetState = new SwerveModuleState();
//        targetState = new SwerveModuleState();
//    }
//
//    @Override
//    public void readPeriodic() {
//        currentState = new AdvancedSwerveModuleState(
//                drivePosition,
//                driveVelocity,
//                driveAcceleration,
//                driveTorqueCurrent,
//                theta,
//                omega
//        );
//
//        // General Logging
//        Logger.recordOutput("Drivetrain/" + moduleName + "/CurrentState", currentState.getCurrentState(Utils.getCurrentTimeSeconds()));
//        Logger.recordOutput("Drivetrain/" + moduleName + "/TargetState", targetState);
//        // Drive Motor Logging
//        Logger.recordOutput("Drivetrain/" + moduleName + "Drivetrain/Position",currentState.position);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Velocity", currentState.velocity);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Acceleration", currentState.acceleration);
//        Logger.recordOutput("Drivetrain/" + moduleName + "DriveMotor/Torque", currentState.torque);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Current", currentState.current);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Force", currentState.force);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/Temperature", driveTemperature.getValue());
//        Logger.recordOutput("Drivetrain/" + moduleName + "/DriveMotor/SupplyVoltage", driveVoltage.getValue());
//        //Steer Motor Logging
//        Logger.recordOutput("Drivetrain/" + moduleName + "/SteerMotor/Voltage", steerMotor.getBusVoltage());
//        Logger.recordOutput("Drivetrain/" + moduleName + "/SteerMotor/Current", steerMotor.getOutputCurrent());
//        Logger.recordOutput("Drivetrain/" + moduleName + "/SteerMotor/Temperature", steerMotor.getMotorTemperature());
//        //Encoder Logging
//        Logger.recordOutput("Drivetrain/" + moduleName + "/Theta", currentState.theta);
//        Logger.recordOutput("Drivetrain/" + moduleName + "/Omega", currentState.omega);
//    }
//
//    @Override
//    public void writePeriodic() {
//        double targetVelocity = targetState.speedMetersPerSecond / (Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES) * Math.PI);
//        targetVelocity = DrivetrainConstants.DRIVE_GEAR_RATIO * targetVelocity;
//        double targetAcceleration = (targetState.speedMetersPerSecond - prevTargetState.speedMetersPerSecond) / Robot.PERIOD;
//        driveControl = new VelocityVoltage(targetVelocity, targetAcceleration,true,12,1,false,false,false);
//        driveMotor.setControl(driveControl);
//        driveMotorSim.setInputVoltage((targetVelocity/DrivetrainConstants.MAX_RPM_FOC) * 12);
//
//        double steerOutput = steerController.calculate(BaseStatusSignal.getLatencyCompensatedValue(theta,omega),targetState.angle.getRotations());
//        steerOutput = MathUtil.clamp(steerOutput, -12,12);
//        steerMotor.setVoltage(steerOutput);
//        steerMotorSim.setInputVoltage(steerOutput);
//    }
//
//    @Override
//    public void simulatePeriodic() {
//        driveMotorSim.update(Robot.PERIOD);
//        steerMotorSim.update(Robot.PERIOD);
//
//        TalonFXSimState driveSimState = driveMotor.getSimState();
//        driveSimState.setRotorVelocity(driveMotorSim.getAngularVelocityRadPerSec() * 2 * Math.PI);
//        driveSimState.setRawRotorPosition(driveMotorSim.getAngularPositionRotations());
//
//        CANcoderSimState steerSimState = encoder.getSimState();
//        steerSimState.setVelocity(steerMotorSim.getAngularVelocityRadPerSec() * 2 * Math.PI);
//    }
//
//    @Override
//    public void setTargetState(SwerveModuleState targetState) {
//        this.targetState = targetState;
//        this.targetState = SwerveModuleState.optimize(targetState, Rotation2d.fromRadians(getCurrentState().theta));
//    }
//
//    @Override
//    public AdvancedSwerveModuleState getCurrentState() {
//        return currentState;
//    }

}
