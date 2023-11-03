package org.frcteam2910.c2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * The default drive command that will run if no other command is currently running.
 */
public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final double ROTATION_COEFFICIENT = 0.6;

    public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.085;

    /** Getting the x velocity from the product of the joysticks (Left Y) and the max velocity */
    private final DoubleSupplier xVelocitySupplier;
    /** Getting y velocity from the product of the joysticks (Left X) and the max velocity */
    private final DoubleSupplier yVelocitySupplier;
    /** Getting the angular velocity from the product of the joysticks (Right X) and the max angular velocity. */
    private final DoubleSupplier angularVelocitySupplier;

    private final BooleanSupplier joystickButton;

    /**
     * The default drive command constructor.
     *
     * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules.
     * @param xVelocitySupplier Gets the joystick input (Left Y) and multiplies it by the max drive velocity.
     * @param yVelocitySupplier Gets the joystick input (Left X) and multiplies it by the max drive velocity.
     * @param angularVelocitySupplier Gets the joystick (Right X) input and multiplies it by the max angular velocity.
     */
    public DefaultDriveCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier,
            BooleanSupplier joystickButton) {
        drivetrain = drivetrainSubsystem;

        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        this.joystickButton = joystickButton;

        addRequirements(drivetrainSubsystem); // Required for default commands of a subsystem.
    }

    @Override
    public void execute() {
        // Running the lambda statements and getting the velocities.
        double xVelocity;
        double yVelocity;
        double angularVelocity;

        xVelocity = xVelocitySupplier.getAsDouble();

        yVelocity = yVelocitySupplier.getAsDouble();

        angularVelocity =
                angularVelocitySupplier.getAsDouble() * (joystickButton.getAsBoolean() ? 1.0 : ROTATION_COEFFICIENT);

        Logger.getInstance()
                .recordOutput("Drive/Rotation offset", drivetrain.getAngularVelocity() * ANGULAR_VELOCITY_COEFFICIENT);

        // Creating a new Chassis speed.
        ChassisSpeeds targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity,
                drivetrain
                        .getPose()
                        .getRotation()
                        .plus(new Rotation2d(drivetrain.getAngularVelocity() * ANGULAR_VELOCITY_COEFFICIENT)));

        // Setting our robot velocity to chassis velocity.
        drivetrain.setTargetVelocity(targetVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetVelocity(new ChassisSpeeds()); // Stop motors
    }
}
