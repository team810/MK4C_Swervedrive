package frc.robot.subsystems.drivetrain;

import frc.robot.lib.AdvancedSubsystem;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private static final DrivetrainSubsystem INSTANCE = new DrivetrainSubsystem();
    public DrivetrainSubsystem() {

    }
    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    public static DrivetrainSubsystem getInstance() {
        return INSTANCE;
    }
}
