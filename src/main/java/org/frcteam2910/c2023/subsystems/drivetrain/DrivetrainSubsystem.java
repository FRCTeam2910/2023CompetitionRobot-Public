package org.frcteam2910.c2023.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.Utils;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.c2023.config.DrivetrainConfiguration;
import org.frcteam2910.c2023.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinator for the drivetrain. Speaks to the Gyro and the Swerve Modules
 */
public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * Horizontal distance between the module centers
     */
    private static final double WHEELBASE_METERS = Units.inchesToMeters(22.75);
    /**
     * Vertical distance between the module centers
     */
    private static final double TRACK_METERS = Units.inchesToMeters(20.75);

    private static final double ESTIMATION_COEFFICIENT = 0.025;

    /**
     * Max transitional velocity of the drivetrain
     */
    private final double maxVelocityMetersPerSec;
    /**
     * Max angular velocity of the drivetrain
     */
    private final double maxAngularVelocityRadPerSec;

    /**
     * Gyro IO interface. Used to update gyro values within the IO inner class
     */
    private final GyroIO gyroIO;

    private final VisionSubsystem vision;

    /**
     * Class with the data read from the gyro implementation.
     */
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

    /**
     * Holds our swerve modules. Contains the speed and angle logic for each module.
     * <p>
     * [0 = front left, 1 = front right, 2 = back left, 3 = back right]
     */
    private final SwerveModule[] swerveModules;
    /**
     * We use this to calculate angular calculations based off of the swerve module locations on the bot
     */
    private final SwerveDriveKinematics kinematics;

    /**
     * Allows us to track the robot's position using the swerve module positions on the robot and the current values of each module.
     */
    private final SwerveDriveOdometry odometry;

    private final SwerveDrivePoseEstimator estimator;

    private final SwerveModulePosition[] swerveModulePositions;
    private final OdometryUpdateThread odometryUpdateThread;

    /**
     * The target speed of the entire robot (not just individual modules). We use this object to calculate the target speeds for each individual module.
     */
    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private final PIDController xController = new PIDController(8, 0, 0);
    private final PIDController yController = new PIDController(8, 0, 0);
    private final PIDController rotationController = new PIDController(1.5, 0, 0);

    private final PPHolonomicDriveController swerveFollower =
            new PPHolonomicDriveController(xController, yController, rotationController);

    private boolean shouldUseVisionData = true;
    private boolean goodToEject = false;

    private class OdometryUpdateThread extends Thread {
        private BaseStatusSignalValue[] allSignals;
        public int successfulDataAcquisitions = 0;
        public int failedDataAcquisitions = 0;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryUpdateThread() {
            ArrayList<BaseStatusSignalValue> signalsList = new ArrayList<>();
            allSignals = new BaseStatusSignalValue[(4 * 4) + 2];
            for (int i = 0; i < 4; i++) {
                signalsList.addAll(Arrays.asList(swerveModules[i].getSignals()));
            }

            signalsList.addAll(Arrays.asList(gyroIO.getSignals()));
            allSignals = signalsList.toArray(new BaseStatusSignalValue[0]);
        }

        public void run() {
            for (var sig : allSignals) {
                if (sig instanceof StatusSignalValue) {
                    ((StatusSignalValue<?>) sig).setUpdateFrequency(250);
                }
            }
            while (true) {
                var status = BaseStatusSignalValue.waitForAll(0.1, allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);
                if (status.isOK()) {
                    successfulDataAcquisitions++;
                } else {
                    failedDataAcquisitions++;
                }

                synchronized (swerveModules) {
                    synchronized (swerveModulePositions) {
                        /* Now update odometry */
                        for (int i = 0; i < 4; ++i) {
                            swerveModules[i].updateInputs();
                            swerveModulePositions[i] = swerveModules[i].getPosition();
                        }
                    }
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed

                synchronized (gyroIO) {
                    synchronized (gyroInputs) {
                        gyroIO.updateInputs(gyroInputs);
                    }
                }
                synchronized (odometry) {
                    synchronized (swerveModulePositions) {
                        synchronized (gyroInputs) {
                            odometry.update(Rotation2d.fromRadians(gyroInputs.yaw), swerveModulePositions);
                        }
                    }
                }

                synchronized (estimator) {
                    synchronized (swerveModulePositions) {
                        synchronized (gyroInputs) {
                            estimator.update(Rotation2d.fromRadians(gyroInputs.yaw), swerveModulePositions);
                        }
                    }
                }
            }
        }

        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDataAcquisitions() {
            return successfulDataAcquisitions;
        }

        public int getFailedDataAcquisitions() {
            return failedDataAcquisitions;
        }
    }

    /**
     * Brains of the drivetrain subsystem. Initializes the swerve drive IOs, swerve drive modules, and swerve drive kinematics and odometry.
     */
    public DrivetrainSubsystem(
            GyroIO gyroIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO,
            VisionSubsystem vision,
            DrivetrainConfiguration configuration) {
        maxVelocityMetersPerSec = frontLeftSwerveModuleIO.getMaxVelocity();
        maxAngularVelocityRadPerSec = maxVelocityMetersPerSec / Math.hypot(WHEELBASE_METERS / 2, TRACK_METERS / 2);

        if (gyroIO == null) gyroIO = new GyroIOSim(this);
        this.gyroIO = gyroIO;
        this.vision = vision;

        swerveModules = new SwerveModule[] {
            new SwerveModule(frontLeftSwerveModuleIO, "FrontLeft"),
            new SwerveModule(frontRightSwerveModuleIO, "FrontRight"),
            new SwerveModule(backLeftSwerveModuleIO, "BackLeft"),
            new SwerveModule(backRightSwerveModuleIO, "BackRight")
        };
        swerveModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
        var frontLeftLocation = new Translation2d(WHEELBASE_METERS / 2, TRACK_METERS / 2);
        var frontRightLocation = new Translation2d(WHEELBASE_METERS / 2, -TRACK_METERS / 2);
        var backLeftLocation = new Translation2d(-WHEELBASE_METERS / 2, TRACK_METERS / 2);
        var backRightLocation = new Translation2d(-WHEELBASE_METERS / 2, -TRACK_METERS / 2);
        kinematics =
                new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 0.5));

        odometryUpdateThread = new OdometryUpdateThread();
        odometryUpdateThread.start();
    }

    @Override
    public void periodic() {
        Logger.getInstance()
                .processInputs("Drive/Gyro", gyroInputs); // Logging the gyro scope readings. Goes to AdvantageScope

        SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];

        swerveModuleStates = kinematics.toSwerveModuleStates(
                targetVelocity); // Using the target chassis speed (speed of the entire robot), we calculate the
        // angle
        // and speed for each swerve drive module by creating separate swerve drive module
        // states.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                maxVelocityMetersPerSec); // Making sure that one module isn't going faster than it's allowed max
        // speed.

        // Making sure that the swerve module wheel can get to the desired state in the fastest way possible.
        // Linking
        // this with the PIDController's continuous input means that the module's angle motor will never turn a
        // wheel
        // more than 90 degrees.
        // SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];
        //        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        synchronized (swerveModules) {
            for (int i = 0; i < optimizedSwerveModuleStates.length; i++) {
                optimizedSwerveModuleStates[i] =
                        SwerveModuleState.optimize(swerveModuleStates[i], swerveModules[i].getPosition().angle);
                swerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);
            }
        }

        Logger.getInstance().recordOutput("Drive/Angle", getPose().getRotation().getRadians());
        Logger.getInstance().recordOutput("Drive/ModuleStates", swerveModuleStates);
        Logger.getInstance().recordOutput("Drive/TargetChassisVelocity", new double[] {
            targetVelocity.vxMetersPerSecond, targetVelocity.vyMetersPerSecond, targetVelocity.omegaRadiansPerSecond
        });
        Logger.getInstance().recordOutput("Drive/ModuleStates", swerveModuleStates); // Logging each module state
        Logger.getInstance()
                .recordOutput(
                        "Drive/OptimizedModuleStates",
                        optimizedSwerveModuleStates); // Logging each optimized module state
        // Figures out the current location and rotation of the robot on the field.

        Logger.getInstance().recordOutput("Drive/UsingVision?", shouldUseVisionData);
        if (shouldUseVisionData) {
            synchronized (estimator) {
                for (int i = 0; i < vision.getVisionOdometry().size(); i++) {
                    estimator.addVisionMeasurement(
                            vision.getVisionOdometry().get(i).getPose(),
                            vision.getVisionOdometry().get(i).getTimestamp(),
                            VecBuilder.fill(
                                    vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
                                    vision.getMinDistance(i) * ESTIMATION_COEFFICIENT,
                                    5.0));
                }
            }
        }

        vision.setReferencePose(getPose());
        Logger.getInstance().recordOutput("Drive/Pose", getPose()); // Logging the pose data
        synchronized (estimator) {
            Logger.getInstance().recordOutput("Drive/EstimatedPose", estimator.getEstimatedPosition());
        }
        synchronized (odometry) {
            Logger.getInstance().recordOutput("Drive/OdometryPose", odometry.getPoseMeters());
        }
        Logger.getInstance().recordOutput("Drive/OdometryThreadLoop", odometryUpdateThread.getTime());
    }

    /**
     * Sets the desired drivetrain speed. The drivetrain will attempt to achieve this speed
     *
     * @param targetVelocity The target chassis speed
     */
    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    /**
     * Gets the field-oriented position of the robot.
     * <p>
     * Using the <a href= https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html>WPILIB Coordinate System</a>.
     *
     * @return The pose of the robot on the field.
     */
    public SwerveDrivePoseEstimator getOdometry() {
        synchronized (estimator) {
            return estimator;
        }
    }

    public Pose2d getPose() {
        return getPose(false);
    }

    public Pose2d getPose(boolean visionAngle) {
        synchronized (estimator) {
            synchronized (odometry) {
                return new Pose2d(
                        estimator.getEstimatedPosition().getX(),
                        estimator.getEstimatedPosition().getY(),
                        visionAngle
                                ? estimator.getEstimatedPosition().getRotation()
                                : odometry.getPoseMeters().getRotation());
            }
        }
    }

    public PPHolonomicDriveController getSwerveFollower() {
        return swerveFollower;
    }

    public SwerveModule[] getSwerveModules() {
        synchronized (swerveModules) {
            return swerveModules;
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetPose() {
        Pose2d pose = getPose();
        resetPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
    }

    public void resetPose(Pose2d poseMeters) {
        synchronized (odometry) {
            synchronized (swerveModulePositions) {
                synchronized (gyroInputs) {
                    odometry.resetPosition(new Rotation2d(gyroInputs.yaw), swerveModulePositions, poseMeters);
                }
            }
        }
        synchronized (estimator) {
            synchronized (swerveModulePositions) {
                synchronized (gyroInputs) {
                    estimator.resetPosition(new Rotation2d(gyroInputs.yaw), swerveModulePositions, poseMeters);
                }
            }
        }
    }

    public GyroIOInputsAutoLogged getGyroInputs() {
        synchronized (gyroInputs) {
            return gyroInputs;
        }
    }

    public Pose2d getProjectedPose(double latency) {
        ChassisSpeeds currentVelocity = kinematics.toChassisSpeeds(swerveModuleStates);

        double xSinceLastPose = currentVelocity.vxMetersPerSecond * latency;
        double ySinceLastPose = currentVelocity.vyMetersPerSecond * latency;
        double angleSinceLastPose = currentVelocity.omegaRadiansPerSecond * latency;

        Twist2d poseChanges = new Twist2d(xSinceLastPose, ySinceLastPose, angleSinceLastPose);
        return getPose().exp(poseChanges);
    }

    public double getMaxVelocityMetersPerSec() {
        return maxVelocityMetersPerSec;
    }

    public double getMaxAngularVelocityRadPerSec() {
        return maxAngularVelocityRadPerSec;
    }

    public void setShouldUseVisionData(boolean shouldUseVisionData) {
        this.shouldUseVisionData = shouldUseVisionData;
    }

    public boolean isGoodToEject() {
        return goodToEject;
    }

    public void setGoodToEject(boolean goodToEject) {
        this.goodToEject = goodToEject;
    }

    public double getAngularVelocity() {
        synchronized (gyroInputs) {
            return gyroInputs.angularVelocity;
        }
    }
}
