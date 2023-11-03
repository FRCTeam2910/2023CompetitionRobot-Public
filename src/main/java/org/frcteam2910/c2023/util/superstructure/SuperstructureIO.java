package org.frcteam2910.c2023.util.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
    default void updateInputs(SuperstructureIOInputs inputs) {}

    @AutoLog
    class SuperstructureIOInputs {
        public boolean homeButtonPressed;
        public boolean brakeButtonPressed;
    }
}
