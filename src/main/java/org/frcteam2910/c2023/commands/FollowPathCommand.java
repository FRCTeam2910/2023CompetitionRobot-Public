package org.frcteam2910.c2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends CommandBase {
    private static final double ACCELERATION_COEFFICIENT = 0.2;

    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(2);

    /**
     * Timer object
     */
    private final Timer timer = new Timer();

    /**
     * Drivetrain object to access subsystem.
     */
    private DrivetrainSubsystem drivetrainSubsystem;
    /**
     * Path Planner trajectory to follow
     */
    private PathPlannerTrajectory trajectory;

    private PathPlannerTrajectory.PathPlannerState previousState;

    /**
     * The auto balance on charge station command constructor.
     *
     * @param drivetrainSubsystem The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public FollowPathCommand(DrivetrainSubsystem drivetrainSubsystem, PathPlannerTrajectory trajectory) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.trajectory = trajectory;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // Reset and begin timer
        this.timer.reset();
        this.timer.start();
        // Logger.getInstance().recordOutput("Drivetrain/Trajectory", trajectory);
        // Get initial state of path
        previousState = trajectory.getInitialState();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        // Determine desired state based on where the robot should be at the current time in the path
        PathPlannerTrajectory.PathPlannerState desiredState =
                (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
        // desiredState.poseMeters.getTranslation().minus(previousState.poseMeters.getTranslation()).getAngle();
        // Transform state if necessary
        desiredState = PathPlannerTrajectory.transformStateForAlliance(desiredState, DriverStation.getAlliance());
        Rotation2d heading = desiredState.poseMeters.getRotation();
        // Calculate our target velocity based on current pose and desired state
        ChassisSpeeds chassisSpeeds =
                drivetrainSubsystem.getSwerveFollower().calculate(drivetrainSubsystem.getPose(), desiredState);

        //
        //        chassisSpeeds.vxMetersPerSecond += desiredState.velocityMetersPerSecond * heading.getCos();
        //        chassisSpeeds.vyMetersPerSecond += desiredState.velocityMetersPerSecond * heading.getSin();
        //        chassisSpeeds.omegaRadiansPerSecond += desiredState.holonomicAngularVelocityRadPerSec;
        //
        chassisSpeeds.vxMetersPerSecond +=
                desiredState.accelerationMetersPerSecondSq * heading.getCos() * ACCELERATION_COEFFICIENT;
        chassisSpeeds.vyMetersPerSecond +=
                desiredState.accelerationMetersPerSecondSq * heading.getSin() * ACCELERATION_COEFFICIENT;

        drivetrainSubsystem.setTargetVelocity(chassisSpeeds);
        Logger.getInstance()
                .recordOutput(
                        "Desired Auto Pose",
                        new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop(); // Stop timer
        drivetrainSubsystem.setTargetVelocity(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        var currentPose = drivetrainSubsystem.getPose();
        var desiredPose = trajectory.getEndState().poseMeters;
        double driveX = drivetrainSubsystem.getPose().getX();
        double driveY = drivetrainSubsystem.getPose().getY();
        double driveRotation = drivetrainSubsystem.getPose().getRotation().getRadians();

        double desiredX = trajectory.getEndState().poseMeters.getX();
        double desiredY = trajectory.getEndState().poseMeters.getY();
        double desiredRotation =
                trajectory.getEndState().poseMeters.getRotation().getRadians();

        double xError = Math.abs(desiredX - driveX);
        double yError = Math.abs(desiredY - driveY);
        double rotationError = Math.abs(desiredRotation - driveRotation);

        return (xError < ALLOWABLE_POSE_ERROR
                        && yError < ALLOWABLE_POSE_ERROR
                        && rotationError < ALLOWABLE_ROTATION_ERROR)
                || timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
