package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This is driving from always blue
 */
public class TelopDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

    public TelopDriveCommand() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION, DrivetrainConstants.MAX_ACCELERATION * 2, 0);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION, DrivetrainConstants.MAX_ACCELERATION * 2, 0);
        omegaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION, DrivetrainConstants.MAX_ACCELERATION * 2, 0);

        addRequirements(DrivetrainSubsystem.getInstance());
    }

    @Override
    public void execute() {
        double invert = 1;
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            invert = -1;
        }
        double verticalVelocity;
        double horizontalVelocity;
        double omegaVelocity;

        horizontalVelocity = -IO.getJoystickValue(Controls.driveYVelocity).get();
        verticalVelocity = -IO.getJoystickValue(Controls.driveXVelocity).get();
        omegaVelocity = -IO.getJoystickValue(Controls.driveOmega).get(); // CCW position so left positive is good

        horizontalVelocity = MathUtil.applyDeadband(horizontalVelocity,.05);
        verticalVelocity = MathUtil.applyDeadband(verticalVelocity,.05);
        omegaVelocity = MathUtil.applyDeadband(omegaVelocity,.05);

        verticalVelocity = verticalVelocity * DrivetrainConstants.MAX_VELOCITY;
        horizontalVelocity = horizontalVelocity * DrivetrainConstants.MAX_VELOCITY;
        omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;

//        verticalVelocity = applyRateLimiting(verticalVelocity,Direction.vertical);
//        horizontalVelocity = applyRateLimiting(horizontalVelocity,Direction.horizontal);
//        omegaVelocity = applyRateLimiting(omegaVelocity,Direction.omega);

        verticalVelocity = verticalVelocity * invert;
        horizontalVelocity = horizontalVelocity * invert;

        ChassisSpeeds targetSpeeds;
        targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
        targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds,DrivetrainSubsystem.getInstance().getCurrentRotation());
        DrivetrainSubsystem.getInstance().setTargetSpeed(targetSpeeds);
    }

    double applyRateLimiting(double limitedRate, Direction direction)
    {
        boolean positive = limitedRate > 0;
        if (positive)
        {
            switch (direction)
            {
                case vertical -> {
                    limitedRate = yLimiter.calculate(limitedRate);
                }
                case horizontal -> {
                    limitedRate = xLimiter.calculate(limitedRate);
                }
                case omega -> {
                    limitedRate = omegaLimiter.calculate(limitedRate);
                }
            }

        }else{
            limitedRate *= -1;
            switch (direction)
            {
                case vertical -> {
                    limitedRate = yLimiter.calculate(limitedRate);
                }
                case horizontal -> {
                    limitedRate = xLimiter.calculate(limitedRate);
                }
                case omega -> {
                    limitedRate = omegaLimiter.calculate(limitedRate);
                }
            }
            limitedRate *= -1;
        }
        return limitedRate;
    }

    @Override
    public boolean isFinished() {

        return !RobotState.isTeleop();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setTargetSpeed(new ChassisSpeeds(0,0,0));
    }

    enum Direction
    {
        vertical,horizontal,omega
    }
}
