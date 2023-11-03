package org.frcteam2910.c2023.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.c2023.Robot;
import org.frcteam2910.c2023.util.ArmPositions;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    // Distance from pivot point to arm base
    public static final double ARM_PIVOT_OFFSET = Units.inchesToMeters(6.75 - 0.125);
    // Distance from origin of robot to pivot point
    public static final Translation2d ORIGIN_PIVOT_OFFSET =
            new Translation2d(-Units.inchesToMeters(10.25), Units.inchesToMeters(13.25));
    public static final double MIN_EXTENSION_LENGTH = Units.inchesToMeters(22.75);
    public static final double MAX_EXTENSION_LENGTH = Units.inchesToMeters(53);
    public static final double MAX_WRIST_ANGLE_RAD = Units.degreesToRadians(113.1);
    public static final double MIN_RETRACTED_WRIST_ANGLE_RAD = Units.degreesToRadians(-73);
    public static final double MIN_EXTENDED_WRIST_ANGLE_RAD = Units.degreesToRadians(-80);
    public static final double MIN_DANGER_ZONE_WRIST_ANGLE = Units.degreesToRadians(60);
    public static final double MIN_EXTENDED_ANGLE_MIN = Units.inchesToMeters(26);
    public static final double MIN_SHOULDER_ANGLE_RAD = 0;
    public static final double MAX_SHOULDER_ANGLE_RAD = Units.degreesToRadians(200);
    private static final double AT_TARGET_THRESHOLD = Units.inchesToMeters(1);
    private static final double AT_ROTATION_THRESHOLD = Units.degreesToRadians(2.5);
    private static final double EXTENSION_FAST_ANGLE_THRESHOLD = Units.degreesToRadians(100);
    private static final double EXTENSION_SLOW_FAST_ANGLE_THRESHOLD = Units.degreesToRadians(70);
    public static final double EXTENSION_DANGER_ZONE = MIN_EXTENSION_LENGTH + Units.inchesToMeters(3);
    public static final double SHOULDER_DANGER_ZONE = Units.degreesToRadians(20);

    private static final double SHOULDER_LOCK_DISTANCE = Math.toRadians(120.0);
    private static final double SHOULDER_LOCK_EXTENSION_DISTANCE = Units.inchesToMeters(28);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private final Mechanism2d targetMech = new Mechanism2d(8, 4);
    private final MechanismRoot2d targetRoot =
            targetMech.getRoot("arm", 4 + ORIGIN_PIVOT_OFFSET.getX(), 0 + ORIGIN_PIVOT_OFFSET.getY());
    private final MechanismLigament2d targetShoulder = targetRoot.append(
            new MechanismLigament2d("targetShoulder", ARM_PIVOT_OFFSET, 270, 10, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d targetArm =
            targetShoulder.append(new MechanismLigament2d("targetArm", 1, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismLigament2d targetWrist = targetArm.append(new MechanismLigament2d("targetWrist", .3, 0));

    private final Mechanism2d currentMech = new Mechanism2d(8, 4);
    private final MechanismRoot2d currentRoot =
            currentMech.getRoot("arm", 4 + ORIGIN_PIVOT_OFFSET.getX(), 0 + ORIGIN_PIVOT_OFFSET.getY());
    private final MechanismLigament2d currentShoulder = targetRoot.append(
            new MechanismLigament2d("currentShoulder", ARM_PIVOT_OFFSET, 270, 10, new Color8Bit(Color.kSilver)));
    private final MechanismLigament2d currentArm =
            currentShoulder.append(new MechanismLigament2d("currentArm", 1, 90, 10, new Color8Bit(Color.kYellowGreen)));
    private final MechanismLigament2d currentWrist =
            currentArm.append(new MechanismLigament2d("currentWrist", .3, 0, 10, new Color8Bit(Color.kAqua)));

    private Pose2d currentPose = new Pose2d();
    private ArmPositions targetPose;

    private boolean zeroed = false;
    private boolean isTeleop = false;
    private boolean isBrakeMode = true;

    public enum ArmStates {
        GROUND_INTAKE,
        PORTAL_INTAKE
    }

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(armInputs);
        Logger.getInstance().processInputs("Arm", armInputs);

        if (Robot.isSimulation() || zeroed) {
            if (targetPose != null) {
                double shoulderTargetAngle = targetPose.getShoulderAngleRad();
                double extensionTargetLength = targetPose.getExtensionLengthMeters();
                double wristTargetAngle = targetPose.getWristAngleRad();

                Logger.getInstance().recordOutput("Arm/PreProcessedShoulderTargetAngle", shoulderTargetAngle);
                Logger.getInstance().recordOutput("Arm/PreProcessedExtensionTargetLength", extensionTargetLength);
                Logger.getInstance().recordOutput("Arm/PreProcessedWristTargetAngle", wristTargetAngle);

                boolean notWithinThreshold = Math.abs(armInputs.shoulderAngleRad - shoulderTargetAngle)
                        > (DriverStation.isTeleop()
                                ? EXTENSION_FAST_ANGLE_THRESHOLD
                                : EXTENSION_SLOW_FAST_ANGLE_THRESHOLD);

                Logger.getInstance().recordOutput("Arm/NotWithinShoulderThreshold", notWithinThreshold);
                Logger.getInstance()
                        .recordOutput("Arm/ShoulderDelta", Math.abs(armInputs.shoulderAngleRad - shoulderTargetAngle));
                if (notWithinThreshold) {
                    extensionTargetLength = MIN_EXTENSION_LENGTH;
                }

                boolean targetInDanger = isInDangerZone(targetPose);
                if (targetInDanger
                        && armInputs.wristAngleRad < MIN_DANGER_ZONE_WRIST_ANGLE
                        && armInputs.shoulderAngleRad < Units.degreesToRadians(15.0)) {
                    extensionTargetLength = Math.max(extensionTargetLength, EXTENSION_DANGER_ZONE);
                }

                boolean currentInDanger = isInDangerZone(new ArmPositions(
                        armInputs.shoulderAngleRad, armInputs.extensionPositionMeters, armInputs.wristAngleRad));

                if (currentInDanger || targetInDanger)
                    wristTargetAngle =
                            MathUtil.clamp(wristTargetAngle, MIN_DANGER_ZONE_WRIST_ANGLE, MAX_WRIST_ANGLE_RAD);

                if (extensionTargetLength < MIN_EXTENDED_ANGLE_MIN
                        || armInputs.extensionPositionMeters < MIN_EXTENDED_ANGLE_MIN) {
                    wristTargetAngle = Math.max(MIN_RETRACTED_WRIST_ANGLE_RAD, wristTargetAngle);
                } else {
                    wristTargetAngle = Math.max(MIN_EXTENDED_WRIST_ANGLE_RAD, wristTargetAngle);
                }

                if (armInputs.shoulderAngleRad - targetPose.getShoulderAngleRad() > SHOULDER_LOCK_DISTANCE
                        && armInputs.extensionPositionMeters > MAX_EXTENSION_LENGTH - SHOULDER_LOCK_EXTENSION_DISTANCE
                        && DriverStation.isTeleop()) {
                    shoulderTargetAngle = ArmPoseConstants.SECONDARY_L3_CONE.getShoulderAngleRad();
                }

                if (isTeleop != DriverStation.isTeleop()) {
                    isTeleop = DriverStation.isTeleop();
                    io.setShoulderAccelerationConstraint(isTeleop);
                }

                //                if (shoulderTargetAngle > armInputs.shoulderAngleRad) {
                //                    if ((shoulderTargetAngle - armInputs.shoulderAngleRad)
                //                            > ((Math.abs(armInputs.shoulderAngularVelocityRadPerSec) *
                // armInputs.shoulderAngularVelocityRadPerSec)
                //                                    / (2 * ArmIOFalcon500.SHOULDER_SLOW_ACCELERATION))) {
                //                        io.setShoulderAccelerationConstraint(true);
                //                    } else {
                //                        io.setShoulderAccelerationConstraint(false);
                //                    }
                //                    io.setShoulderAccelerationConstraint(true);
                //                } else {
                //                    if (-armInputs.shoulderAngularVelocityRadPerSec >=
                // ArmIOFalcon500.SHOULDER_VELOCITY) {
                //                        io.setShoulderAccelerationConstraint(true);
                //                    } else {
                //                        if (Math.abs((shoulderTargetAngle - armInputs.shoulderAngleRad))
                //                                < (Math.pow(armInputs.shoulderAngularVelocityRadPerSec, 2)
                //                                        / (2 * ArmIOFalcon500.SHOULDER_SLOW_ACCELERATION))) {
                //                            io.setShoulderAccelerationConstraint(true);
                //                        } else {
                //                            io.setShoulderAccelerationConstraint(false);
                //                        }
                //                    }
                //                    io.setShoulderAccelerationConstraint(false);
                //                }

                ArmPositions validStates =
                        clampValidState(shoulderTargetAngle, extensionTargetLength, wristTargetAngle);

                io.setTargetWristAngle(validStates.getWristAngleRad());
                io.setTargetShoulderAngle(validStates.getShoulderAngleRad());
                io.setTargetExtensionLength(validStates.getExtensionLengthMeters());

                if (isTargetStowed()) {
                    if (atTarget()) {
                        io.setExtensionCurrentLimit(false);
                    }
                } else {
                    io.setExtensionCurrentLimit(true);
                }

                targetShoulder.setAngle(270 + Units.radiansToDegrees(shoulderTargetAngle));
                targetArm.setLength(extensionTargetLength);
                targetWrist.setAngle(Units.radiansToDegrees(wristTargetAngle));

                Logger.getInstance().recordOutput("Arm/TargetInDanger", targetInDanger);
                Logger.getInstance().recordOutput("Arm/CurrentInDanger", currentInDanger);
                Logger.getInstance().recordOutput("Arm/ShoulderTargetAngle", shoulderTargetAngle);
                Logger.getInstance().recordOutput("Arm/ExtensionTargetLength", extensionTargetLength);
                Logger.getInstance().recordOutput("Arm/WristTargetAngle", wristTargetAngle);

            } else {
                io.setWristVoltage(0);
                io.setExtensionVoltage(0);
                io.setShoulderVoltage(0);
            }
        }

        currentShoulder.setAngle(270 + Units.radiansToDegrees(armInputs.shoulderAngleRad));
        currentArm.setLength(armInputs.extensionPositionMeters);
        currentWrist.setAngle(Units.radiansToDegrees(armInputs.wristAngleRad));

        Logger.getInstance().recordOutput("Arm/Current Pose", currentPose);
        Logger.getInstance().recordOutput("Arm/Mechanism", targetMech);
    }

    public void setTargetPose(ArmPositions armPositions) {
        this.targetPose = armPositions;
    }

    public void setTargetPose(Pose3d targetPose, Pose2d currentPose) {
        double x;
        if (targetPose.getRotation().getAngle() > -Math.PI / 2
                && targetPose.getRotation().getAngle() < Math.PI / 2) {
            x = -currentPose.getX() + targetPose.getX() - ORIGIN_PIVOT_OFFSET.getX();
        } else {
            x = currentPose.getX() - targetPose.getX() - ORIGIN_PIVOT_OFFSET.getX();
        }
        double y = targetPose.getZ() - ORIGIN_PIVOT_OFFSET.getY();

        this.targetPose = calcTargetPose(new Pose2d(x, y, new Rotation2d(0)));
    }

    public void calcComponentPoses() {
        // Pose3d for component simulation
        Pose3d phaseOne = new Pose3d(
                ORIGIN_PIVOT_OFFSET.getX(),
                0,
                ORIGIN_PIVOT_OFFSET.getY(),
                new Rotation3d(0, -armInputs.shoulderAngleRad, 0));
        Pose3d phaseTwo = new Pose3d(
                (ORIGIN_PIVOT_OFFSET.getX() + armSegmentCalc()[0] * 0.5),
                0,
                (ORIGIN_PIVOT_OFFSET.getY() + armSegmentCalc()[1] * 0.5),
                new Rotation3d(0, -armInputs.shoulderAngleRad, 0));
        Pose3d phaseThree = new Pose3d(
                (ORIGIN_PIVOT_OFFSET.getX() + armSegmentCalc()[0]),
                0,
                (ORIGIN_PIVOT_OFFSET.getY() + armSegmentCalc()[1]),
                new Rotation3d(0, -armInputs.shoulderAngleRad, 0));
        Pose3d wrist = new Pose3d(
                ORIGIN_PIVOT_OFFSET.getX() + currentPose.getX() + 0.05,
                0,
                ORIGIN_PIVOT_OFFSET.getY() + currentPose.getY(),
                new Rotation3d(
                        0, Units.degreesToRadians(currentPose.getRotation().getDegrees() + 90), 0));

        Logger.getInstance().recordOutput("Arm/Arm Phase One", phaseOne);
        Logger.getInstance().recordOutput("Arm/Arm Phase Two", phaseTwo);
        Logger.getInstance().recordOutput("Arm/Arm Phase Three", phaseThree);
        Logger.getInstance().recordOutput("Arm/Wrist Pose3d", wrist);
    }

    /**
     * @return Pose2d of wrist from origin of robot and angle from ground (X, Y, Theta)
     **/
    private Pose2d calcCurrentPose(double armLengthMeters, double shoulderAngleRad, double wristAngleRad) {
        Translation2d wristCoord;
        Rotation2d wristAngle;

        double wristShoulderDist = Math.sqrt(Math.pow(ARM_PIVOT_OFFSET, 2) + Math.pow(armLengthMeters, 2));

        double vectorAngle = shoulderAngleRad - Math.asin(ARM_PIVOT_OFFSET / wristShoulderDist);

        // Calculate vector components
        double x = Math.cos(vectorAngle) * wristShoulderDist + ORIGIN_PIVOT_OFFSET.getX();
        double y = Math.sin(vectorAngle) * wristShoulderDist + ORIGIN_PIVOT_OFFSET.getY();

        // Apply origin pivot offset
        wristCoord = new Translation2d(x, y);

        wristAngle = new Rotation2d(shoulderAngleRad + wristAngleRad);
        return new Pose2d(wristCoord, wristAngle);
    }

    /**
     * @param target pose of wrist location and rotation
     * @return Pose2d of measurements required to get to a set point [Shoulder Angle, Arm Length, Wrist Angle (Rad)]
     **/
    private ArmPositions calcTargetPose(Pose2d target) {
        double x = target.getX() - ORIGIN_PIVOT_OFFSET.getX();
        double y = target.getY() - ORIGIN_PIVOT_OFFSET.getY();
        double wristShoulderDist = Math.hypot(x, y);
        double targetExtensionLength = Math.sqrt(Math.pow(wristShoulderDist, 2) - Math.pow(ARM_PIVOT_OFFSET, 2));
        double targetShoulderAngle = Math.asin(ARM_PIVOT_OFFSET / wristShoulderDist);
        targetShoulderAngle -= Math.atan2(x, y);
        targetShoulderAngle += Units.degreesToRadians(90);
        double wristAngle = target.getRotation().getRadians() - targetShoulderAngle;

        return new ArmPositions(targetShoulderAngle, targetExtensionLength, wristAngle);
    }

    private boolean checkValidState(double targetShoulderAngle, double targetExtensionLength, double wristAngle) {
        if (wristAngle > MAX_WRIST_ANGLE_RAD) {
            return false;
        }
        if (targetExtensionLength < MIN_EXTENDED_ANGLE_MIN && wristAngle < MIN_RETRACTED_WRIST_ANGLE_RAD) {
            return false;
        }
        if (targetExtensionLength > MIN_EXTENDED_ANGLE_MIN && wristAngle < MIN_EXTENDED_WRIST_ANGLE_RAD) {
            return false;
        }
        if (targetShoulderAngle < MIN_SHOULDER_ANGLE_RAD || targetShoulderAngle > MAX_SHOULDER_ANGLE_RAD) {
            return false;
        }
        if (targetExtensionLength < MIN_EXTENSION_LENGTH || targetExtensionLength > MAX_EXTENSION_LENGTH) {
            return false;
        }
        return true;
    }

    private ArmPositions clampValidState(double targetShoulderAngle, double targetExtensionLength, double wristAngle) {
        targetShoulderAngle = MathUtil.clamp(targetShoulderAngle, MIN_SHOULDER_ANGLE_RAD, MAX_SHOULDER_ANGLE_RAD);
        targetExtensionLength = MathUtil.clamp(targetExtensionLength, MIN_EXTENSION_LENGTH, MAX_EXTENSION_LENGTH);
        wristAngle = MathUtil.clamp(wristAngle, MIN_EXTENDED_WRIST_ANGLE_RAD, MAX_WRIST_ANGLE_RAD);
        return new ArmPositions(targetShoulderAngle, targetExtensionLength, wristAngle);
    }

    /**
     * @param length calculated length of arm to reach target
     * @return arm length clamped down to min and max length of telescoping arm (0.578, 1.3275)
     */
    public double applyArmClamp(double length) {
        return MathUtil.clamp(length, MIN_EXTENSION_LENGTH, MAX_EXTENSION_LENGTH);
    }

    /**
     * @return x and y coordinates of telescoping section of arm without the fixed minimum length
     */
    private double[] armSegmentCalc() {
        double tele = armInputs.extensionPositionMeters - MIN_EXTENSION_LENGTH;
        double x = Math.cos(armInputs.shoulderAngleRad) * tele;
        double y = Math.sin(armInputs.shoulderAngleRad) * tele;
        return new double[] {x, y};
    }

    public void setJointVoltage(ArmJoint joint, double voltage) {
        switch (joint) {
            case SHOULDER:
                io.setShoulderVoltage(voltage);
                break;
            case EXTENSION:
                io.setExtensionVoltage(voltage);
                break;
            case WRIST:
                io.setWristVoltage(voltage);
                break;
        }
    }

    public void setZeroed(boolean zero) {
        this.zeroed = zero;
    }

    public boolean isZeroed() {
        return zeroed;
    }
    /**
     * @return radians per second for the shoulder and wrist, meters per second for the extension
     */
    public double getVelocity(ArmJoint joint) {
        switch (joint) {
            case SHOULDER:
                return armInputs.shoulderAngularVelocityRadPerSec;
            case EXTENSION:
                return armInputs.extensionVelocityMetersPerSec;
            case WRIST:
                return armInputs.wristAngularVelocityRadPerSec;
            default:
                return 0.0;
        }
    }

    public boolean atTarget() {
        if (targetPose == null) {
            return false;
        }
        return Math.abs(targetPose.getShoulderAngleRad() - armInputs.shoulderAngleRad) < AT_ROTATION_THRESHOLD
                && Math.abs(targetPose.getExtensionLengthMeters() - armInputs.extensionPositionMeters)
                        < AT_TARGET_THRESHOLD
                && Math.abs(targetPose.getWristAngleRad() - armInputs.wristAngleRad) < AT_ROTATION_THRESHOLD;
    }

    public void setAllSensorPositionsFromMeasurement(ArmPositions positions) {
        io.setSensorPositionFromMeasurement(ArmJoint.SHOULDER, positions.getShoulderAngleRad());
        io.setSensorPositionFromMeasurement(ArmJoint.EXTENSION, positions.getExtensionLengthMeters());
        io.setSensorPositionFromMeasurement(ArmJoint.WRIST, positions.getWristAngleRad());
    }

    public void setSensorPositionFromMeasurement(ArmJoint joint, double position) {
        io.setSensorPositionFromMeasurement(joint, position);
    }

    public void toggleBrakeMode() {
        isBrakeMode = !isBrakeMode;
        io.setNeutralMode(isBrakeMode);
    }

    private boolean isInDangerZone(ArmPositions positions) {
        return positions.getShoulderAngleRad() < SHOULDER_DANGER_ZONE
                && positions.getExtensionLengthMeters() < EXTENSION_DANGER_ZONE;
    }

    private boolean isTargetStowed() {
        return targetPose.equals(ArmPoseConstants.STOW);
    }
}
