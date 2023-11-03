package org.frcteam2910.c2023.config;

public interface RobotConstants {
    DrivetrainConfiguration getDrivetrainConfiguration();

    PortConfiguration getPortConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case LOKI_2023_ONE:
                return new Loki();
            case PHANTOM_2023_TWO:
                return new Phantom();
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new Phantom();
        }
    }
}
