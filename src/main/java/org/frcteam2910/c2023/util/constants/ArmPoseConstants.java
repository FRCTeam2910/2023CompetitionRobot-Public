package org.frcteam2910.c2023.util.constants;

import edu.wpi.first.math.util.Units;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.util.ArmPositions;

public final class ArmPoseConstants {
    private static final double SECONDARY_L2_SHOULDER_OFFSET = Units.degreesToRadians(20.0);
    private static final double SECONDARY_L3_SHOULDER_OFFSET = Units.degreesToRadians(10.0);

    private static final double SUPERCHARGE_L2_SHOULDER_OFFSET = Units.degreesToRadians(2.0);
    private static final double SUPERCHARGE_L3_SHOULDER_OFFSET = Units.degreesToRadians(1.0);

    // L1 arm poses
    public static final ArmPositions L1_CONE = new ArmPositions(
            Units.degreesToRadians(26.82), Units.inchesToMeters(24.51575), Units.degreesToRadians(-73.358));
    public static final ArmPositions L1_CONE_SHOOT = new ArmPositions(
            Units.degreesToRadians(1.6905), Units.inchesToMeters(22.83465), Units.degreesToRadians(109.608));
    public static final ArmPositions L1_CUBE =
            new ArmPositions(Units.degreesToRadians(1.6905), Units.inchesToMeters(23.0), Units.degreesToRadians(65.0));

    // L2 arm poses
    public static final ArmPositions L2_CONE =
            new ArmPositions(Units.degreesToRadians(138.0), Units.inchesToMeters(34.0), Units.degreesToRadians(56.41));
    public static final ArmPositions SECONDARY_L2_CONE = new ArmPositions(
            L2_CONE.getShoulderAngleRad() - SECONDARY_L2_SHOULDER_OFFSET,
            L2_CONE.getExtensionLengthMeters(),
            L2_CONE.getWristAngleRad());
    public static final ArmPositions L2_CONE_SUPERCHARGED = new ArmPositions(
            L2_CONE.getShoulderAngleRad() - SUPERCHARGE_L2_SHOULDER_OFFSET,
            L2_CONE.getExtensionLengthMeters(),
            L2_CONE.getWristAngleRad());
    public static final ArmPositions L2_CUBE_BACK = new ArmPositions(
            Units.degreesToRadians(143.0), Units.inchesToMeters(33.4252), Units.degreesToRadians(112.6915));
    public static final ArmPositions L2_CUBE_FRONT =
            new ArmPositions(Units.degreesToRadians(35.35), Units.inchesToMeters(39.84), Units.degreesToRadians(41.96));

    // L3 arm poses
    public static final ArmPositions L3_CONE =
            new ArmPositions(Units.degreesToRadians(141.0), Units.inchesToMeters(53), Units.degreesToRadians(45));
    public static final ArmPositions SECONDARY_L3_CONE = new ArmPositions(
            L3_CONE.getShoulderAngleRad() - SECONDARY_L3_SHOULDER_OFFSET,
            L3_CONE.getExtensionLengthMeters(),
            L3_CONE.getWristAngleRad());
    public static final ArmPositions L3_CONE_SUPERCHARGED = new ArmPositions(
            L3_CONE.getShoulderAngleRad() - SUPERCHARGE_L3_SHOULDER_OFFSET,
            L3_CONE.getExtensionLengthMeters(),
            L3_CONE.getWristAngleRad());
    public static final ArmPositions L3_CUBE = new ArmPositions(
            Units.degreesToRadians(142.3663), Units.inchesToMeters(52.48031), Units.degreesToRadians(112.7227));

    // Portal arm poses
    public static final ArmPositions PORTAL_CONE = new ArmPositions(
            Units.degreesToRadians(111.5), Units.inchesToMeters(43.50), Units.degreesToRadians(88.7846));
    public static final ArmPositions PORTAL_CUBE =
            new ArmPositions(Units.degreesToRadians(59), Units.inchesToMeters(43), Units.degreesToRadians(-50));

    // Ground poses
    public static final ArmPositions GROUND_CONE_FLAT =
            new ArmPositions(Units.degreesToRadians(8), Units.inchesToMeters(29.13386), Units.degreesToRadians(-61.50));
    public static final ArmPositions GROUND_CONE_UPRIGHT = new ArmPositions(
            Units.degreesToRadians(182.0), Units.inchesToMeters(22.835), Units.degreesToRadians(24.1472));
    public static final ArmPositions GROUND_CUBE = new ArmPositions(
            Units.degreesToRadians(1.6905), Units.inchesToMeters(26.0), Units.degreesToRadians(13.1927));
    public static final ArmPositions SINGLE_SUBSTATION_CUBE =
            new ArmPositions(Units.degreesToRadians(33.7), Units.inchesToMeters(22.95), Units.degreesToRadians(44.86));

    // Stow positions
    public static final ArmPositions STOW = new ArmPositions(
            Units.degreesToRadians(1.6905), Units.inchesToMeters(23.0), Units.degreesToRadians(109.608));
    public static final ArmPositions MANUAL_HOME_POSITION = new ArmPositions(
            Units.degreesToRadians(1.158), Units.inchesToMeters(22.73), Units.degreesToRadians(109.53));

    public static final ArmPositions ARM_UP =
            new ArmPositions(Units.degreesToRadians(90), Units.inchesToMeters(23.0), Units.degreesToRadians(90));

    public static final ArmPositions LONG_GROUND_CUBE = new ArmPositions(
            Units.degreesToRadians(1.6905), ArmSubsystem.MAX_EXTENSION_LENGTH, Units.degreesToRadians(13.1927));

    public static final ArmPositions L1_CUBE_BACK = new ArmPositions(
            Units.degreesToRadians(180), ArmSubsystem.MIN_EXTENSION_LENGTH, ArmSubsystem.MAX_WRIST_ANGLE_RAD);

    public static final ArmPositions LONG_L1_CUBE_BACK =
            new ArmPositions(Units.degreesToRadians(180), Units.inchesToMeters(36.0), Units.degreesToRadians(100.0));
    public static final ArmPositions BUMP_LONG_L1_CUBE_BACK =
            new ArmPositions(Units.degreesToRadians(180), Units.inchesToMeters(48.0), Units.degreesToRadians(109.608));
}
