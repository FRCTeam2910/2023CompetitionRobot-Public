package org.frcteam2910.c2023.util.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.frcteam2910.c2023.subsystems.intake.IntakeSubsystem;
import org.frcteam2910.c2023.util.GamePiece;
import org.frcteam2910.c2023.util.GridLevel;
import org.frcteam2910.c2023.util.OperatorDashboard;
import org.littletonrobotics.junction.Logger;

public class WaypointConstants {
    public static final Pose3d centralWaypoint =
            new Pose3d(Units.inchesToMeters(219), Units.inchesToMeters(175), 0.0, new Rotation3d());

    public static Pose3d rightCommunity =
            new Pose3d(Units.inchesToMeters(180), Units.inchesToMeters(40), 0.0, new Rotation3d());

    public static final double COLUMN_WAYPOINT_DISTANCE = Units.inchesToMeters(24.0);

    public static Pose3d portalWaypoint;

    public static Pose3d columnOneWaypoint;

    public static Pose3d columnTwoWaypoint;

    public static Pose3d columnThreeWaypoint;

    public static Pose3d columnFourWaypoint;

    public static Pose3d columnFiveWaypoint;

    public static Pose3d columnSixWaypoint;

    public static Pose3d columnSevenWaypoint;

    public static Pose3d columnEightWaypoint;

    public static Pose3d columnNineWaypoint;

    public static Pose2d[] columnWaypoints;

    public static void update() {
        portalWaypoint = new Pose3d(
                FieldConstants.PORTAL_TAG_X + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.PORTAL_TAG_Y,
                0.0,
                new Rotation3d());

        rightCommunity = new Pose3d(Units.inchesToMeters(180), Units.inchesToMeters(40), 0.0, new Rotation3d());

        columnOneWaypoint = new Pose3d(
                FieldConstants.COLUMN_ONE[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_ONE[0].getY(),
                0.0,
                new Rotation3d());

        columnTwoWaypoint = new Pose3d(
                FieldConstants.COLUMN_TWO[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_TWO[0].getY(),
                0.0,
                new Rotation3d());

        columnThreeWaypoint = new Pose3d(
                FieldConstants.COLUMN_THREE[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_THREE[0].getY(),
                0.0,
                new Rotation3d());

        columnFourWaypoint = new Pose3d(
                FieldConstants.COLUMN_FOUR[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_FOUR[0].getY(),
                0.0,
                new Rotation3d());

        columnFiveWaypoint = new Pose3d(
                FieldConstants.COLUMN_FIVE[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_FIVE[0].getY(),
                0.0,
                new Rotation3d());

        columnSixWaypoint = new Pose3d(
                FieldConstants.COLUMN_SIX[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_SIX[0].getY(),
                0.0,
                new Rotation3d());

        columnSevenWaypoint = new Pose3d(
                FieldConstants.COLUMN_SEVEN[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_SEVEN[0].getY(),
                0.0,
                new Rotation3d());

        columnEightWaypoint = new Pose3d(
                FieldConstants.COLUMN_EIGHT[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_EIGHT[0].getY(),
                0.0,
                new Rotation3d());

        columnNineWaypoint = new Pose3d(
                FieldConstants.COLUMN_NINE[0].getX() + COLUMN_WAYPOINT_DISTANCE,
                FieldConstants.COLUMN_NINE[0].getY(),
                0.0,
                new Rotation3d());

        columnWaypoints = new Pose2d[] {
            WaypointConstants.columnOneWaypoint.toPose2d(),
            WaypointConstants.columnTwoWaypoint.toPose2d(),
            WaypointConstants.columnThreeWaypoint.toPose2d(),
            WaypointConstants.columnFourWaypoint.toPose2d(),
            WaypointConstants.columnFiveWaypoint.toPose2d(),
            WaypointConstants.columnSixWaypoint.toPose2d(),
            WaypointConstants.columnSevenWaypoint.toPose2d(),
            WaypointConstants.columnEightWaypoint.toPose2d(),
            WaypointConstants.columnNineWaypoint.toPose2d()
        };

        Logger.getInstance().recordOutput("FieldConstants/field column 1", FieldConstants.COLUMN_ONE[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 2", FieldConstants.COLUMN_TWO[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 3", FieldConstants.COLUMN_THREE[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 4", FieldConstants.COLUMN_FOUR[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 5", FieldConstants.COLUMN_FIVE[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 6", FieldConstants.COLUMN_SIX[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 7", FieldConstants.COLUMN_SEVEN[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 8", FieldConstants.COLUMN_EIGHT[0]);
        Logger.getInstance().recordOutput("FieldConstants/field column 9", FieldConstants.COLUMN_NINE[0]);
    }

    public static int getClosestNode(Pose2d currentPose, OperatorDashboard dashboard, IntakeSubsystem intakeSubsystem) {
        int offset = dashboard.getScoringPositionOffset();
        GamePiece gamePiece = intakeSubsystem.getTargetPiece();
        GridLevel level = dashboard.getSelectedGridLevel();
        int closestWaypoint = 0;

        double currentY = currentPose.getY()
                + (intakeSubsystem.getTargetPiece() == GamePiece.CONE
                        ? intakeSubsystem.getConeOffset().getY()
                        : 0.0)
                + Units.inchesToMeters(dashboard.getTranslationOffset());

        Logger.getInstance()
                .recordOutput(
                        "Auto Align/currentY in getClosestNode()",
                        new Pose2d(currentPose.getX(), currentY, currentPose.getRotation()));

        for (int i = 0; i < columnWaypoints.length; i++) {
            if (Math.abs(currentY - columnWaypoints[i].getY())
                    < Math.abs(currentY - columnWaypoints[closestWaypoint].getY())) {
                if (isValidScoringPosition(i, gamePiece, level)) {
                    closestWaypoint = i;
                }
            }
        }
        if (!dashboard.isOffsetChangePositive()) {
            while (!isValidScoringPosition(closestWaypoint + offset, gamePiece, level)) {
                if (gamePiece == GamePiece.CONE) {
                    if (closestWaypoint + offset < 0) {
                        offset = -closestWaypoint;
                    } else {
                        offset = offset - 1;
                    }
                } else {
                    if (closestWaypoint + offset < 1) {
                        offset = 1 - closestWaypoint;
                    } else {
                        offset = offset - 1;
                    }
                }
            }
        }
        if (dashboard.isOffsetChangePositive()) {
            while (!isValidScoringPosition(closestWaypoint + offset, gamePiece, level)) {
                if (gamePiece == GamePiece.CONE) {
                    if (closestWaypoint + offset > 8) {
                        offset = 8 - closestWaypoint;
                    } else {
                        offset = offset + 1;
                    }
                } else {
                    if (closestWaypoint + offset > 7) {
                        offset = 7 - closestWaypoint;
                    } else {
                        offset = offset + 1;
                    }
                }
            }
        }
        dashboard.setScoringPositionOffset(offset);
        return MathUtil.clamp(closestWaypoint + offset, 0, columnWaypoints.length - 1);
    }

    public static Pose2d getPoseByWaypoint(int waypoint) {
        return columnWaypoints[waypoint];
    }

    public static int getClosestTagByWaypoint(int waypoint) {
        switch (waypoint) {
            case 0:
            case 1:
            case 2:
                return DriverStation.getAlliance() == DriverStation.Alliance.Red ? 1 : 6;
            case 3:
            case 4:
            case 5:
                return DriverStation.getAlliance() == DriverStation.Alliance.Red ? 2 : 7;
            case 6:
            case 7:
            case 8:
            default:
                return DriverStation.getAlliance() == DriverStation.Alliance.Red ? 3 : 8;
        }
    }

    public static boolean isValidScoringPosition(int column, GamePiece gamePiece, GridLevel level) {
        if (level == GridLevel.LOW) {
            return true;
        } else {
            switch (column) {
                case 0:
                case 2:
                case 3:
                case 5:
                case 6:
                case 8:
                    return gamePiece == GamePiece.CONE;
                case 1:
                case 4:
                case 7:
                    return gamePiece == GamePiece.CUBE;
                default:
                    return false;
            }
        }
    }
}
