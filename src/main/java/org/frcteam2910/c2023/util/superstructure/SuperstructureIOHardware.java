package org.frcteam2910.c2023.util.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;
import org.frcteam2910.c2023.config.PortConfiguration;

public class SuperstructureIOHardware implements SuperstructureIO {
    private final DigitalInput homeButton;
    private final DigitalInput brakeButton;

    public SuperstructureIOHardware(PortConfiguration configuration) {
        homeButton = new DigitalInput(configuration.homeInputID);
        brakeButton = new DigitalInput(configuration.brakeInputID);
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        inputs.homeButtonPressed = !homeButton.get();
        inputs.brakeButtonPressed = !brakeButton.get();
    }
}
