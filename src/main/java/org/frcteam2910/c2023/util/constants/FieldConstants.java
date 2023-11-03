package org.frcteam2910.c2023.util.constants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static final double FIELD_LENGTH_M = Units.feetToMeters(54.0);
    public static AprilTagFieldLayout BLUE_FIELD_LAYOUT;
    public static AprilTagFieldLayout RED_FIELD_LAYOUT;
    private static DriverStation.Alliance storedAlliance = DriverStation.Alliance.Invalid;

    static {
        try {
            BLUE_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            BLUE_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            RED_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            RED_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static final double LOW_CONE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(6.28);
    public static final double MID_CONE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(8.42);
    public static final double HIGH_CONE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(25.45);

    public static final double LOW_CUBE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(7.14);
    public static final double MID_CUBE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(8.655);
    public static final double HIGH_CUBE_X_CENTER_FROM_APRILTAG = Units.inchesToMeters(28.54);

    public static final double MID_CONE_NODE_HEIGHT_METERS = Units.inchesToMeters(34.0);
    public static final double MID_CUBE_NODE_HEIGHT_METERS = Units.inchesToMeters(23.5);
    public static final double HIGH_CONE_NODE_HEIGHT_METERS = Units.inchesToMeters(46.0);
    public static final double HIGH_CUBE_NODE_HEIGHT_METERS = Units.inchesToMeters(35.5);

    public static final double Y_BETWEEN_COLUMNS_METERS = Units.inchesToMeters(22.0);

    public static double PORTAL_TAG_X;
    public static double PORTAL_TAG_Y;
    public static double LEFT_TAG_X;
    public static double LEFT_TAG_Y;
    public static double MIDDLE_TAG_X;
    public static double MIDDLE_TAG_Y;
    public static double RIGHT_TAG_X;
    public static double RIGHT_TAG_Y;

    public static Pose3d PORTAL;

    public static Pose3d[] COLUMN_ONE;
    public static Pose3d[] COLUMN_TWO;
    public static Pose3d[] COLUMN_THREE;
    public static Pose3d[] COLUMN_FOUR;
    public static Pose3d[] COLUMN_FIVE;
    public static Pose3d[] COLUMN_SIX;
    public static Pose3d[] COLUMN_SEVEN;
    public static Pose3d[] COLUMN_EIGHT;
    public static Pose3d[] COLUMN_NINE;

    public static final Pose3d[] CHARGING_STATION = {
        new Pose3d(Units.inchesToMeters(114.74), Units.inchesToMeters(155.39), 0.0, new Rotation3d()), // bottom left
        new Pose3d(Units.inchesToMeters(162.74), Units.inchesToMeters(155.39), 0.0, new Rotation3d()), // top left
        new Pose3d(Units.inchesToMeters(114.74), Units.inchesToMeters(59.39), 0.0, new Rotation3d()), // top right
        new Pose3d(Units.inchesToMeters(114.74), Units.inchesToMeters(59.39), 0.0, new Rotation3d()) // bottom right
    };

    public static final Pose3d[] LOADING_ZONE = {
        new Pose3d(Units.inchesToMeters(385.36), Units.inchesToMeters(312), 0.0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(385.36), Units.inchesToMeters(263.43), 0.0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(515.75), Units.inchesToMeters(263.43), 0.0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(515.75), Units.inchesToMeters(212.93), 0.0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(648), Units.inchesToMeters(212.93), 0.0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(648), Units.inchesToMeters(312), 0.0, new Rotation3d())
    };

    public static AprilTagFieldLayout getFieldLayout() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return BLUE_FIELD_LAYOUT;
        } else {
            return RED_FIELD_LAYOUT;
        }
    }

    public static void update() {
        if (DriverStation.getAlliance() != storedAlliance) {
            storedAlliance = DriverStation.getAlliance();
            System.out.println(storedAlliance);

            PORTAL_TAG_X = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 5 : 4)
                    .get()
                    .getX();
            PORTAL_TAG_Y = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 5 : 4)
                    .get()
                    .getY();
            LEFT_TAG_X = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 1 : 6)
                    .get()
                    .getX();
            LEFT_TAG_Y = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 1 : 6)
                    .get()
                    .getY();
            MIDDLE_TAG_X = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 2 : 7)
                    .get()
                    .getX();
            MIDDLE_TAG_Y = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 2 : 7)
                    .get()
                    .getY();
            RIGHT_TAG_X = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 3 : 8)
                    .get()
                    .getX();
            RIGHT_TAG_Y = getFieldLayout()
                    .getTagPose(storedAlliance == DriverStation.Alliance.Red ? 3 : 8)
                    .get()
                    .getY();

            PORTAL = new Pose3d(FieldConstants.PORTAL_TAG_X, FieldConstants.PORTAL_TAG_Y, 0.0, new Rotation3d());

            COLUMN_ONE = new Pose3d[] {
                new Pose3d(
                        LEFT_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_TWO = new Pose3d[] {
                new Pose3d(LEFT_TAG_X + LOW_CUBE_X_CENTER_FROM_APRILTAG, LEFT_TAG_Y, 0.0, new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - MID_CUBE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y,
                        MID_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - HIGH_CUBE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y,
                        HIGH_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_THREE = new Pose3d[] {
                new Pose3d(
                        LEFT_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        LEFT_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        LEFT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_FOUR = new Pose3d[] {
                new Pose3d(
                        MIDDLE_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_FIVE = new Pose3d[] {
                new Pose3d(MIDDLE_TAG_X + LOW_CUBE_X_CENTER_FROM_APRILTAG, MIDDLE_TAG_Y, 0.0, new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - MID_CUBE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y,
                        MID_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - HIGH_CUBE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y,
                        HIGH_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_SIX = new Pose3d[] {
                new Pose3d(
                        MIDDLE_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        MIDDLE_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        MIDDLE_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_SEVEN = new Pose3d[] {
                new Pose3d(
                        RIGHT_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y + Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_EIGHT = new Pose3d[] {
                new Pose3d(RIGHT_TAG_X + LOW_CUBE_X_CENTER_FROM_APRILTAG, RIGHT_TAG_Y, 0.0, new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - MID_CUBE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y,
                        MID_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - HIGH_CUBE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y,
                        HIGH_CUBE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            COLUMN_NINE = new Pose3d[] {
                new Pose3d(
                        RIGHT_TAG_X + LOW_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        0.0,
                        new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - MID_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        MID_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d()),
                new Pose3d(
                        RIGHT_TAG_X - HIGH_CONE_X_CENTER_FROM_APRILTAG,
                        RIGHT_TAG_Y - Y_BETWEEN_COLUMNS_METERS,
                        HIGH_CONE_NODE_HEIGHT_METERS,
                        new Rotation3d())
            };

            WaypointConstants.update();
        }
    }
}
