package org.frcteam2910.c2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.frcteam2910.c2023.util.constants.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Command for autonomously balancing on charging station.
 */
public class AutoBalanceOnChargeStationCommand extends CommandBase {
    /**
     * Drivetrain object to access subsystem.
     */
    private final DrivetrainSubsystem drivetrainSubsystem;
    /**
     * PID controller which uses the error between the robot's angle and 0 degrees to output a desired x-pose to drive to.
     */
    private final PIDController chargingStationPID = new PIDController(2.5, 0, 0);
    /**
     * PID controller which uses the error between the robot's current x-pose and the pose setpoint to output a field-relative x-velocity.
     */
    private final PIDController xController = new PIDController(1, 0, 0);

    /**
     * The auto balance on charge station command constructor.
     *
     * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules.
     */
    public AutoBalanceOnChargeStationCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // Set the setpoint
        chargingStationPID.setSetpoint(0);
    }

    @Override
    public void execute() {
        // Determine the robot's orientation and utilize the appropriate data based on that.
        Rotation2d currentAngle = drivetrainSubsystem.getPose().getRotation();
        Rotation2d approximateAngle = new Rotation2d();
        //        int multiplesOfNinetyInAngle = (int) Math.round(currentAngle / Math.toRadians(90));
        //        int approximateAngle = 90 * multiplesOfNinetyInAngle;
        // Logger.getInstance().recordOutput("Drive/Approximate angle", approximateAngle);

        //        switch (approximateAngle) {
        //            case 0:
        //                driveUpTheChargeStation(true, 1);
        //                break;
        //            case 90:
        //                driveUpTheChargeStation(false, 1);
        //                break;
        //            case 180:
        //                driveUpTheChargeStation(true, -1);
        //                break;
        //            case 270:
        //                driveUpTheChargeStation(false, -1);
        //                break;
        //            default:
        //            drivetrainSubsystem.setTargetVelocity(new ChassisSpeeds());
        //        }
        if (Math.abs(currentAngle.minus(Rotation2d.fromDegrees(0)).getRadians()) < Math.PI / 4) {
            approximateAngle = Rotation2d.fromDegrees(0);
            driveUpTheChargeStation(true, -1);
        }
        if (Math.abs(currentAngle.minus(Rotation2d.fromDegrees(90)).getRadians()) < Math.PI / 4) {
            approximateAngle = Rotation2d.fromDegrees(90);
            driveUpTheChargeStation(false, -1);
        }
        if (Math.abs(currentAngle.minus(Rotation2d.fromDegrees(180)).getRadians()) < Math.PI / 4) {
            approximateAngle = Rotation2d.fromDegrees(180);
            driveUpTheChargeStation(true, 1);
        }
        if (Math.abs(currentAngle.minus(Rotation2d.fromDegrees(270)).getRadians()) < Math.PI / 4) {
            approximateAngle = Rotation2d.fromDegrees(270);
            driveUpTheChargeStation(false, 1);
        }
        Logger.getInstance().recordOutput("Drive/Appx Angle", approximateAngle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setTargetVelocity(new ChassisSpeeds()); // Stop motors
    }

    public void driveUpTheChargeStation(boolean isInStraightOrientation, int isPositive) {
        double currentAngle;
        if (isInStraightOrientation) {
            currentAngle = drivetrainSubsystem.getGyroInputs().pitch;
        } else {
            currentAngle = drivetrainSubsystem.getGyroInputs().roll;
        }
        if (Math.abs(currentAngle) < Constants.CHARGING_STATION_ALLOWABLE_ANGLE_ERROR) {
            drivetrainSubsystem.setTargetVelocity(new ChassisSpeeds());
        } else {
            drivetrainSubsystem.setTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    chargingStationPID.calculate(currentAngle) * isPositive,
                    0,
                    0,
                    drivetrainSubsystem.getPose().getRotation()));
        }
        //
        //        else {
        //            xController.setSetpoint(truncatePoseSetpoint(
        //                    isRobotForward * chargingStationPID.calculate(currentAngle) +
        // Constants.CHARGING_STATION_X_CENTER));
        //        }
        //        drivetrainSubsystem.setTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
        //                xController.calculate(drivetrainSubsystem.getPose().getX()),
        //                0,
        //                0,
        //                drivetrainSubsystem.getPose().getRotation()));
    }

    /**
     * Truncates the pose setpoint if it is above or below the maximum or minimum poses
     *
     * @param desiredPose The coordinator between the gyro and the swerve modules.
     */
    public double truncatePoseSetpoint(double desiredPose) {
        return Math.min(
                Math.max(desiredPose, Constants.CHARGING_STATION_MINIMUM_X_POSE),
                Constants.CHARGING_STATION_MAXIMUM_X_POSE);
    }
}
