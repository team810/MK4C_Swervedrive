package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This is driving from always blue
 */
public class TelopDriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter omegaLimiter;

    boolean omegaControl = true;

    public TelopDriveCommand() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_THEORETICAL_ACCELERATION);
        omegaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

        addRequirements(DrivetrainSubsystem.getInstance());
    }

    @Override
    public void execute() {
        if (IO.getButtonValue(Controls.thetaOmegaToggle).getAsBoolean()) {
            omegaControl = !omegaControl;
        }

        double verticalVelocity;
        double horizontalVelocity;
        double invert = 1;

        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Red)
        {
            invert = invert * -1;
        }

        horizontalVelocity = -IO.getJoystickValue(Controls.driveYVelocity).get();
        verticalVelocity = -IO.getJoystickValue(Controls.driveXVelocity).get();

        horizontalVelocity = horizontalVelocity * invert;
        verticalVelocity = verticalVelocity * invert;

        horizontalVelocity = MathUtil.applyDeadband(horizontalVelocity, .05);
        verticalVelocity = MathUtil.applyDeadband(verticalVelocity, .05);

        verticalVelocity = verticalVelocity * DrivetrainConstants.MAX_VELOCITY;
        horizontalVelocity = horizontalVelocity * DrivetrainConstants.MAX_VELOCITY;

        verticalVelocity = xLimiter.calculate(verticalVelocity);
        horizontalVelocity = yLimiter.calculate(horizontalVelocity);

        if (omegaControl) {
            double omegaVelocity;

            omegaVelocity = -IO.getJoystickValue(Controls.driveOmega).get(); // CCW position so left positive is good
            omegaVelocity = MathUtil.applyDeadband(omegaVelocity, .05);
            omegaVelocity = omegaVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY;
            omegaVelocity = omegaLimiter.calculate(omegaVelocity);

            ChassisSpeeds targetSpeeds;
            targetSpeeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omegaVelocity);
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, DrivetrainSubsystem.getInstance().getPose().getRotation());
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityFOC);
            DrivetrainSubsystem.getInstance().setVelocityFOC(targetSpeeds);
        }else{
            DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.VelocityThetaControlFOC);
            double xThetaInput = -MathUtil.applyDeadband(IO.getJoystickValue(Controls.driveThetaX).get(), .4);
            double yThetaInput = -MathUtil.applyDeadband(IO.getJoystickValue(Controls.driveThetaY).get(), .4);
            Rotation2d targetRot = new Rotation2d(xThetaInput,yThetaInput);
            if (xThetaInput == 0 && yThetaInput == 0)
            {
                targetRot = DrivetrainSubsystem.getInstance().getPose().getRotation();
            }

            DrivetrainSubsystem.getInstance().setVelocityThetaControlFOC(horizontalVelocity,verticalVelocity,targetRot);
        }

    }

    @Override
    public boolean isFinished() {
        return !RobotState.isTeleop();
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setVelocityFOC(new ChassisSpeeds());
        DrivetrainSubsystem.getInstance().setControlMode(DrivetrainSubsystem.ControlMethods.off);
    }
}
