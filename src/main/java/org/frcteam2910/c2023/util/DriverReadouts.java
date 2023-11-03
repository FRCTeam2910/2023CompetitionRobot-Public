package org.frcteam2910.c2023.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.frcteam2910.c2023.RobotContainer;
import org.frcteam2910.c2023.util.constants.Constants;

public class DriverReadouts {

    public DriverReadouts(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUTS_TAB_NAME);

        tab.add("Autonomous Mode", PathChooser.getModeChooser()).withSize(5, 2).withPosition(9, 0);
    }
}
