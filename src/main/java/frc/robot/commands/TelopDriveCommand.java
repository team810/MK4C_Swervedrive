package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        omegaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

        addRequirements(DrivetrainSubsystem.getInstance());
    }

    @Override
    public void execute() {
        double verticalVelocity;
        double horizontalVelocity;
        double omegaVelocity;

        horizontalVelocity = -IO.getJoystickValue(Controls.driveYVelocity).get();
        verticalVelocity = -IO.getJoystickValue(Controls.driveXVelocity).get();
        omegaVelocity = -IO.getJoystickValue(Controls.driveOmega).get(); // CCW position so left positive is good

        horizontalVelocity = MathUtil.applyDeadband(horizontalVelocity, .05);
        verticalVelocity = MathUtil.applyDeadband(verticalVelocity, .05);
        omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .05);

        verticalVelocity = verticalVelocity * DrivetrainConstants.MAX_VELOCITY;
        horizontalVelocity = horizontalVelocity * DrivetrainConstants.MAX_VELOCITY;
        omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;

        verticalVelocity = xLimiter.calculate(verticalVelocity);
        horizontalVelocity = yLimiter.calculate(horizontalVelocity);
        omegaVelocity = omegaLimiter.calculate(omegaVelocity);

        ChassisSpeeds targetSpeeds;
        targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
        targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, DrivetrainSubsystem.getInstance().getPose().getRotation());
        DrivetrainSubsystem.getInstance().setTargetSpeed(targetSpeeds);
    }

    @Override
    public boolean isFinished() {

        return !RobotState.isTeleop();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setTargetSpeed(new ChassisSpeeds(0, 0, 0));
    }
}
