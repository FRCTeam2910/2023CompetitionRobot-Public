package org.frcteam2910.c2023.config;

import org.frcteam2910.c2023.Robot;
import org.frcteam2910.c2023.util.MacAddressUtil;
import org.frcteam2910.c2023.util.constants.Constants;

import static org.frcteam2910.c2023.util.MacAddressUtil.*;

public enum RobotIdentity {
    LOKI_2023_ONE,
    PHANTOM_2023_TWO,
    ROBOT_2022,
    SIMULATION;

    public static RobotIdentity getIdentity() {
        if (Robot.isReal()) {
            String mac = getMACAddress();
            if (!mac.equals("")) {
                if (mac.equals(MacAddressUtil.ROBOT_ONE_MAC) || mac.equals(SECONDARY_ROBOT_ONE_MAC)) {
                    return LOKI_2023_ONE;
                } else if (mac.equals(MacAddressUtil.ROBOT_TWO_MAC)) {
                    return PHANTOM_2023_TWO;
                }
            }
            if (Constants.COMPETITION_ROBOT == LOKI_2023_ONE) {
                return LOKI_2023_ONE;
            } else {
                return PHANTOM_2023_TWO;
            }
        } else {
            return SIMULATION;
        }
    }
}
