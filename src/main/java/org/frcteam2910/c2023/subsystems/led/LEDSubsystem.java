package org.frcteam2910.c2023.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    public enum WantedAction {
        DISPLAY_LOW_BATTERY,
        DISPLAY_ROBOT_ZEROED,
        DISPLAY_ROBOT_NOT_ZEROED,
        DISPLAY_ROBOT_ARM_ZEROED,
        DISPLAY_ROBOT_ARM_NOT_ZEROED,
        DISPLAY_INITIALIZE,
        DISPLAY_GETTING_CUBE,
        DISPLAY_GETTING_CONE,
        DISPLAY_GETTING_CUBE_FLASHING,
        DISPLAY_GETTING_CONE_FLASHING,
        DISPLAY_GETTING_CUBE_FLASHING_WHITE,
        DISPLAY_GETTING_CONE_FLASHING_WHITE,
        DISPLAY_LIMELIGHT_MIMICRY,
        DISPLAY_IS_DOING_CHARGING_STATION,
        DISPLAY_SUPERSTRUCTURE,
        DISPLAY_OFF,
        DISPLAY_ALIGN_BAD
    }

    private enum SystemState {
        DISPLAYING_LOW_BATTERY,
        DISPLAYING_ROBOT_ZEROED,
        DISPLAYING_ROBOT_NOT_ZEROED,
        DISPLAYING_ROBOT_ARM_ZEROED,
        DISPLAYING_ROBOT_ARM_NOT_ZEROED,
        DISPLAYING_INITIALIZE,
        DISPLAYING_GETTING_CUBE,
        DISPLAYING_GETTING_CONE,
        DISPLAYING_GETTING_CUBE_FLASHING,
        DISPLAYING_GETTING_CONE_FLASHING,
        DISPLAYING_GETTING_CUBE_FLASHING_WHITE,
        DISPLAYING_GETTING_CONE_FLASHING_WHITE,
        DISPLAYING_LIMELIGHT_MIMICRY,
        DISPLAYING_IS_DOING_CHARGING_STATION,
        DISPLAYING_SUPERSTRUCTURE,
        DISPLAYING_OFF,
        DISPLAYING_ALIGN_BAD
    }

    private double stateStartTime;
    private SystemState systemState = SystemState.DISPLAYING_SUPERSTRUCTURE;
    private WantedAction wantedAction = WantedAction.DISPLAY_SUPERSTRUCTURE;
    private LEDState desiredLEDState = new LEDState(0, 0, 0);
    private final LEDTimedState superstructureLEDState = LEDTimedState.StaticLEDState.staticOff;
    private final LEDIO ledIO;

    private void setZeroedCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.ROBOT_ZEROING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setNotZeroedCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.ROBOT_NOT_ZEROING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setLowBatteryCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.BATTERY_LOW.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setSuperstructureLEDCommand(double timeInState) {
        superstructureLEDState.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setInitializeCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.INITIALIZING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setConeCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CONE.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setCubeCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CUBE.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setConeFlashingCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CONE_FLASHING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setCubeFlashingCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CUBE_FLASHING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setConeFlashingWhiteCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CONE_FLASHING_WHITE.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setCubeFlashingWhiteCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.GETTING_CUBE_FLASHING_WHITE.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setArmZeroedCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.ARM_ZEROING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setArmNotZeroedCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.ARM_NOT_ZEROING.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setLimelightMimicryCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.LIMELIGHT_MIMICRY.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setClimbingCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.IS_DOING_CHARGING_STATION.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setOffCommand(double timeInState) {
        LEDTimedState.StaticLEDState.staticOff.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private void setAlignBadCommand(double timeInState) {
        LEDTimedState.BlinkingLEDState.ALIGN_BAD.getCurrentLEDState(desiredLEDState, timeInState);
    }

    private SystemState getStateTransition() {
        switch (wantedAction) {
            case DISPLAY_INITIALIZE:
                return SystemState.DISPLAYING_INITIALIZE;
            case DISPLAY_SUPERSTRUCTURE:
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
            case DISPLAY_LOW_BATTERY:
                return SystemState.DISPLAYING_LOW_BATTERY;
            case DISPLAY_GETTING_CONE:
                return SystemState.DISPLAYING_GETTING_CONE;
            case DISPLAY_GETTING_CUBE:
                return SystemState.DISPLAYING_GETTING_CUBE;
            case DISPLAY_GETTING_CONE_FLASHING:
                return SystemState.DISPLAYING_GETTING_CONE_FLASHING;
            case DISPLAY_GETTING_CUBE_FLASHING:
                return SystemState.DISPLAYING_GETTING_CUBE_FLASHING;
            case DISPLAY_GETTING_CONE_FLASHING_WHITE:
                return SystemState.DISPLAYING_GETTING_CONE_FLASHING_WHITE;
            case DISPLAY_GETTING_CUBE_FLASHING_WHITE:
                return SystemState.DISPLAYING_GETTING_CUBE_FLASHING_WHITE;
            case DISPLAY_ROBOT_ZEROED:
                return SystemState.DISPLAYING_ROBOT_ZEROED;
            case DISPLAY_ROBOT_NOT_ZEROED:
                return SystemState.DISPLAYING_ROBOT_NOT_ZEROED;
            case DISPLAY_ROBOT_ARM_ZEROED:
                return SystemState.DISPLAYING_ROBOT_ARM_ZEROED;
            case DISPLAY_ROBOT_ARM_NOT_ZEROED:
                return SystemState.DISPLAYING_ROBOT_ARM_NOT_ZEROED;
            case DISPLAY_LIMELIGHT_MIMICRY:
                return SystemState.DISPLAYING_LIMELIGHT_MIMICRY;
            case DISPLAY_IS_DOING_CHARGING_STATION:
                return SystemState.DISPLAYING_IS_DOING_CHARGING_STATION;
            case DISPLAY_ALIGN_BAD:
                return SystemState.DISPLAYING_ALIGN_BAD;
            case DISPLAY_OFF:
                return SystemState.DISPLAYING_OFF;
            default:
                System.out.println("Fell through on LED wanted action check: " + wantedAction);
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
        }
    }

    public void setWantedAction(WantedAction wantedAction) {
        this.wantedAction = wantedAction;
    }

    private final LEDIO.LEDIOInputs ledioInputs = new LEDIO.LEDIOInputs();

    public LEDSubsystem(LEDIO ledIO) {
        this.ledIO = ledIO;
    }

    @Override
    public void periodic() {
        var timestamp = Timer.getFPGATimestamp();
        SystemState newState = getStateTransition();
        if (systemState != newState) {
            System.out.println(timestamp + ": LED changed states: " + systemState + " -> " + newState);
            systemState = newState;
            stateStartTime = timestamp;
        }
        double timeInState = timestamp - stateStartTime;
        switch (systemState) {
            case DISPLAYING_SUPERSTRUCTURE:
                setSuperstructureLEDCommand(timeInState);
                break;
            case DISPLAYING_LOW_BATTERY:
                setLowBatteryCommand(timeInState);
                break;
            case DISPLAYING_ROBOT_ZEROED:
                setZeroedCommand(timeInState);
                break;
            case DISPLAYING_ROBOT_NOT_ZEROED:
                setNotZeroedCommand(timeInState);
                break;
            case DISPLAYING_ROBOT_ARM_ZEROED:
                setArmZeroedCommand(timeInState);
                break;
            case DISPLAYING_ROBOT_ARM_NOT_ZEROED:
                setArmNotZeroedCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CONE:
                setConeCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CUBE:
                setCubeCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CONE_FLASHING:
                setConeFlashingCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CUBE_FLASHING:
                setCubeFlashingCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CONE_FLASHING_WHITE:
                setConeFlashingWhiteCommand(timeInState);
                break;
            case DISPLAYING_GETTING_CUBE_FLASHING_WHITE:
                setCubeFlashingWhiteCommand(timeInState);
                break;
            case DISPLAYING_INITIALIZE:
                setInitializeCommand(timeInState);
                break;
            case DISPLAYING_LIMELIGHT_MIMICRY:
                setLimelightMimicryCommand(timeInState);
                break;
            case DISPLAYING_IS_DOING_CHARGING_STATION:
                setClimbingCommand(timeInState);
                break;
            case DISPLAYING_ALIGN_BAD:
                setAlignBadCommand(timeInState);
                break;
            case DISPLAYING_OFF:
                setOffCommand(timeInState);
                break;
            default:
                System.out.println("Fell through on LED commands: " + systemState);
                break;
        }
        ledIO.updateInputs(ledioInputs, desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
        ledIO.setLEDs(desiredLEDState.red, desiredLEDState.green, desiredLEDState.blue);
    }

    public WantedAction getWantedAction() {
        return wantedAction;
    }

    public boolean getIsBadAlign() {
        return wantedAction == WantedAction.DISPLAY_ALIGN_BAD;
    }

    public boolean getIsFlashing() {
        return wantedAction == WantedAction.DISPLAY_GETTING_CONE_FLASHING
                || wantedAction == WantedAction.DISPLAY_GETTING_CUBE_FLASHING
                || wantedAction == WantedAction.DISPLAY_GETTING_CONE_FLASHING_WHITE
                || wantedAction == WantedAction.DISPLAY_GETTING_CUBE_FLASHING_WHITE;
    }
}
