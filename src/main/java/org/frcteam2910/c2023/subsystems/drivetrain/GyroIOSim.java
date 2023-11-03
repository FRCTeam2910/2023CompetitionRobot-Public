package org.frcteam2910.c2023.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class GyroIOSim implements GyroIO {
    DrivetrainSubsystem drivetrainSubsystem;

    private Pose2d simOdometry = new Pose2d();
    double[] lastModulePositionsRad = {0, 0, 0, 0};

    public GyroIOSim(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        calcAngle();
        inputs.yaw = simOdometry.getRotation().getRadians();
    }

    private void calcAngle() {
        drivetrainSubsystem.getSwerveModules();

        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = drivetrainSubsystem.getSwerveModules()[i].getPosition().angle;
        }

        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (drivetrainSubsystem.getSwerveModules()[i].getPosition().distanceMeters - lastModulePositionsRad[i])
                            * Units.inchesToMeters(2),
                    turnPositions[i]);
            lastModulePositionsRad[i] = drivetrainSubsystem.getSwerveModules()[i].getPosition().distanceMeters;
        }

        simOdometry = simOdometry.exp(new Twist2d(
                drivetrainSubsystem.getKinematics().toChassisSpeeds(measuredStatesDiff).vxMetersPerSecond,
                drivetrainSubsystem.getKinematics().toChassisSpeeds(measuredStatesDiff).vyMetersPerSecond,
                drivetrainSubsystem.getKinematics().toChassisSpeeds(measuredStatesDiff).omegaRadiansPerSecond));
    }
}
