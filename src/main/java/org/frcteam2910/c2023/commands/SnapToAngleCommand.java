package org.frcteam2910.c2023.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

public class SnapToAngleCommand extends CommandBase {
    private static final double STATIC_FRICTION_CONSTANT = 0.0;
    public static final double ALLOWABLE_ANGLE_ERROR = Units.degreesToRadians(10);
    private static final double SCORING_VELOCITY_COEFFICIENT = 0.25;
    private static final double INTAKING_VELOCITY_COEFFICIENT = 0.3;

    private static final double MAX_ANGULAR_VELOCITY_PERCENT = 0.3;
    /**
     * Drivetrain object to access subsystem.
     */
    private final DrivetrainSubsystem drivetrain;
    /**
     * Target angle
     */
    private final Supplier<Rotation2d> targetAngle;
    /**
     * PID controller which uses the error between the current angle and setpoint angle to output angular velocity
     */
    private final PIDController snapToAnglePID = new PIDController(4.0, 0, 0.2);

    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;

    private final boolean scoring;
    private final boolean onTheFly;

    private final DoubleSupplier angularVelocitySupplier;

    /**
     * @param drivetrainSubsystem - drivetrain object
     * @param targetAngle         - target angle in radians
     */
    public SnapToAngleCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Rotation2d targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            boolean scoring) {
        this(drivetrainSubsystem, () -> targetAngle, xVelocitySupplier, yVelocitySupplier, scoring);
    }

    public SnapToAngleCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            boolean scoring) {
        this(drivetrainSubsystem, targetAngle, xVelocitySupplier, yVelocitySupplier, scoring, false, () -> 0.0);
    }

    public SnapToAngleCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            Supplier<Rotation2d> targetAngle,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            boolean scoring,
            boolean onTheFly,
            DoubleSupplier angularVelocitySupplier) {
        this.drivetrain = drivetrainSubsystem;
        this.targetAngle = targetAngle;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.scoring = scoring;
        this.onTheFly = onTheFly;
        this.angularVelocitySupplier = angularVelocitySupplier;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        snapToAnglePID.reset();
        snapToAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        double rotationalVelocity = MathUtil.clamp(
                snapToAnglePID.calculate(
                        drivetrain.getPose().getRotation().getRadians(),
                        targetAngle.get().getRadians()),
                -MAX_ANGULAR_VELOCITY_PERCENT * drivetrain.getMaxAngularVelocityRadPerSec(),
                MAX_ANGULAR_VELOCITY_PERCENT * drivetrain.getMaxAngularVelocityRadPerSec());
        rotationalVelocity += Math.copySign(STATIC_FRICTION_CONSTANT, rotationalVelocity);

        double xVelocity;
        double yVelocity;
        if (!onTheFly) {
            if (xVelocitySupplier.getAsDouble() < 0.0) {
                xVelocity = xVelocitySupplier.getAsDouble();
            } else {
                xVelocity = xVelocitySupplier.getAsDouble()
                        * (scoring ? SCORING_VELOCITY_COEFFICIENT : INTAKING_VELOCITY_COEFFICIENT);
            }

            yVelocity = yVelocitySupplier.getAsDouble()
                    * (scoring ? SCORING_VELOCITY_COEFFICIENT : INTAKING_VELOCITY_COEFFICIENT);
        } else {
            xVelocity = xVelocitySupplier.getAsDouble();
            yVelocity = yVelocitySupplier.getAsDouble();
        }

        drivetrain.setTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                rotationalVelocity,
                drivetrain
                        .getPose()
                        .getRotation()
                        .plus(new Rotation2d(
                                drivetrain.getAngularVelocity() * DefaultDriveCommand.ANGULAR_VELOCITY_COEFFICIENT))));
        Logger.getInstance().recordOutput("SnapAnglePIDOutput", rotationalVelocity);
        Logger.getInstance().recordOutput("Drive/TargetAngle", targetAngle.get().getRadians());
    }

    @Override
    public boolean isFinished() {
        return onTheFly && Math.abs(angularVelocitySupplier.getAsDouble()) > 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetVelocity(new ChassisSpeeds());
    }
}
