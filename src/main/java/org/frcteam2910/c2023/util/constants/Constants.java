package org.frcteam2910.c2023.util.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.frcteam2910.c2023.config.RobotIdentity;

public final class Constants {
    public static final String DRIVER_READOUTS_TAB_NAME = "Version 1 Grid";

    public static final int PRIMARY_XBOX_CONTROLLER_PORT = 0;
    public static final int OPERATOR_XBOX_CONTROLLER_PORT = 1;
    // Specific to 2022 CanBus - for testing
    // "rio"
    //    public static final String CAN_BUS = "CANivore";

    public static final double CHARGING_STATION_ALLOWABLE_ANGLE_ERROR = Math.toRadians(1.7);
    public static final double CHARGING_STATION_MAXIMUM_X_POSE = 2.5781;
    public static final double CHARGING_STATION_MINIMUM_X_POSE = 2.3368;
    public static final double CHARGING_STATION_X_CENTER = Units.inchesToMeters(149);

    //    public static final double X_FRONT_PHOTON_OFFSET = Units.inchesToMeters(-8.8696);
    public static final double X_BACK_PHOTON_OFFSET = Units.inchesToMeters(-5.89);
    public static final double Y_PHOTON_OFFSET = Units.inchesToMeters(6.30);
    public static final double Z_PHOTON_OFFSET = Units.inchesToMeters(9.66);
    //    public static final double SMALL_ANGLE_PHOTON_OFFSET = Units.degreesToRadians(20);
    public static final double RIGHT_ANGLE_PHOTON_OFFSET = -Units.degreesToRadians(150.0);
    public static final double LEFT_ANGLE_PHOTON_OFFSET = Units.degreesToRadians(150.0);

    //    public static final Translation3d FRONT_LEFT_PHOTON_OFFSET =
    //            new Translation3d(X_FRONT_PHOTON_OFFSET, Y_PHOTON_OFFSET, Z_PHOTON_OFFSET);
    //    public static final Rotation3d FRONT_LEFT_PHOTON_ANGLE = new Rotation3d(0, 0, SMALL_ANGLE_PHOTON_OFFSET);
    //    public static final Transform3d FRONT_LEFT_PHOTON_POSE =
    //            new Transform3d(FRONT_LEFT_PHOTON_OFFSET, FRONT_LEFT_PHOTON_ANGLE);

    public static final double PHOTON_PITCH_OFFSET = Units.degreesToRadians(-15.0);

    public static final Translation3d BACK_LEFT_PHOTON_OFFSET =
            new Translation3d(X_BACK_PHOTON_OFFSET, Y_PHOTON_OFFSET, Z_PHOTON_OFFSET);
    public static final Rotation3d BACK_LEFT_PHOTON_ANGLE =
            new Rotation3d(0, PHOTON_PITCH_OFFSET, LEFT_ANGLE_PHOTON_OFFSET);
    public static final Transform3d BACK_LEFT_PHOTON_POSE =
            new Transform3d(BACK_LEFT_PHOTON_OFFSET, BACK_LEFT_PHOTON_ANGLE);

    //    public static final Translation3d FRONT_RIGHT_PHOTON_OFFSET =
    //            new Translation3d(X_FRONT_PHOTON_OFFSET, Y_PHOTON_OFFSET * -1, Z_PHOTON_OFFSET);
    //    public static final Rotation3d FRONT_RIGHT_PHOTON_ANGLE = new Rotation3d(0, 0, -SMALL_ANGLE_PHOTON_OFFSET);
    //    public static final Transform3d FRONT_RIGHT_PHOTON_POSE =
    //            new Transform3d(FRONT_RIGHT_PHOTON_OFFSET, FRONT_RIGHT_PHOTON_ANGLE);

    public static final Translation3d BACK_RIGHT_PHOTON_OFFSET =
            new Translation3d(X_BACK_PHOTON_OFFSET, -Y_PHOTON_OFFSET, Z_PHOTON_OFFSET);
    public static final Rotation3d BACK_RIGHT_PHOTON_ANGLE =
            new Rotation3d(0, PHOTON_PITCH_OFFSET, RIGHT_ANGLE_PHOTON_OFFSET);
    public static final Transform3d BACK_RIGHT_PHOTON_POSE =
            new Transform3d(BACK_RIGHT_PHOTON_OFFSET, BACK_RIGHT_PHOTON_ANGLE);

    public static final double X_LIMELIGHT_OFFSET = Units.inchesToMeters(-8.3479);
    public static final double Y_LIMELIGHT_OFFSET = Units.inchesToMeters(5.93);
    public static final double Z_LIMELIGHT_OFFSET = Units.inchesToMeters(9.0814);
    public static final double ANGLE_PITCH_LIMELIGHT_OFFSET = Units.degreesToRadians(10);
    public static final double ANGLE_YAW_LIMELIGHT_OFFSET = Units.degreesToRadians(155);

    public static final Translation3d BACK_LEFT_LIMELIGHT_OFFSET =
            new Translation3d(X_LIMELIGHT_OFFSET, Y_LIMELIGHT_OFFSET, Z_LIMELIGHT_OFFSET);
    public static final Rotation3d BACK_LEFT_LIMELIGHT_ANGLE =
            new Rotation3d(0.0, ANGLE_PITCH_LIMELIGHT_OFFSET, ANGLE_YAW_LIMELIGHT_OFFSET);
    public static final Transform3d BACK_LEFT_LIMELIGHT_POSE =
            new Transform3d(BACK_LEFT_LIMELIGHT_OFFSET, BACK_LEFT_LIMELIGHT_ANGLE);

    public static final Translation3d BACK_RIGHT_LIMELIGHT_OFFSET =
            new Translation3d(X_LIMELIGHT_OFFSET, -Y_LIMELIGHT_OFFSET, Z_LIMELIGHT_OFFSET);
    public static final Rotation3d BACK_RIGHT_LIMELIGHT_ANGLE =
            new Rotation3d(0.0, ANGLE_PITCH_LIMELIGHT_OFFSET, ANGLE_YAW_LIMELIGHT_OFFSET * -1);
    public static final Transform3d BACK_RIGHT_LIMELIGHT_POSE =
            new Transform3d(BACK_RIGHT_LIMELIGHT_OFFSET, BACK_RIGHT_LIMELIGHT_ANGLE);

    public static final RobotIdentity COMPETITION_ROBOT = RobotIdentity.PHANTOM_2023_TWO;
}
