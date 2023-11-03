package org.frcteam2910.c2023;

public class RobotConfiguration {
    private static final RobotType robot = RobotType.ROBOT_SIM;

    public static boolean isTuningMode() {
        return true;
    }

    public static RobotType getRobot() {
        if (Robot.isReal()) {
            if (robot == RobotType.ROBOT_SIM) {
                return RobotType.ROBOT_2023;
            }
        }

        return robot;
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2023:
                return Robot.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_SIM:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }

    public enum RobotType {
        ROBOT_2023,
        ROBOT_SIM
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
