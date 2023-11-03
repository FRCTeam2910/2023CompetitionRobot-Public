package org.frcteam2910.c2023.util.superstructure;

import org.littletonrobotics.junction.Logger;

public class Superstructure {
    private final SuperstructureIO io;
    private final SuperstructureIOInputsAutoLogged superstructureInputs = new SuperstructureIOInputsAutoLogged();

    public Superstructure(SuperstructureIO io) {
        this.io = io;
    }

    public void update() {
        io.updateInputs(superstructureInputs);
        Logger.getInstance().processInputs("Superstructure", superstructureInputs);
    }

    public boolean getHomeButtonPressed() {
        return superstructureInputs.homeButtonPressed;
    }

    public boolean getBrakeButtonPressed() {
        return superstructureInputs.brakeButtonPressed;
    }
}
