package org.frcteam2910.c2023.subsystems.led;

public interface LEDTimedState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements LEDTimedState {
        public static final BlinkingLEDState ARM_ZEROING = new BlinkingLEDState(LEDState.OFF, LEDState.ARM_ZEROED, 0.0);
        public static final BlinkingLEDState ARM_NOT_ZEROING =
                new BlinkingLEDState(LEDState.OFF, LEDState.ARM_NOT_ZEROED, 0.0);
        public static final BlinkingLEDState ROBOT_ZEROING =
                new BlinkingLEDState(LEDState.OFF, LEDState.ROBOT_ZEROED, 1.0);
        public static final BlinkingLEDState ROBOT_NOT_ZEROING =
                new BlinkingLEDState(LEDState.OFF, LEDState.ROBOT_NOT_ZEROED, 1.0);
        public static final BlinkingLEDState INITIALIZING =
                new BlinkingLEDState(LEDState.OFF, LEDState.INITIALIZE, 1.0);
        public static final BlinkingLEDState GETTING_CONE = new BlinkingLEDState(LEDState.OFF, LEDState.GET_CONE, 0.0);
        public static final BlinkingLEDState GETTING_CUBE = new BlinkingLEDState(LEDState.OFF, LEDState.GET_CUBE, 0.0);
        public static final BlinkingLEDState GETTING_CONE_FLASHING =
                new BlinkingLEDState(LEDState.OFF, LEDState.GET_CONE, 0.2);
        public static final BlinkingLEDState GETTING_CUBE_FLASHING =
                new BlinkingLEDState(LEDState.OFF, LEDState.GET_CUBE, 0.2);
        public static final BlinkingLEDState GETTING_CONE_FLASHING_WHITE =
                new BlinkingLEDState(LEDState.WOULD_EJECT, LEDState.GET_CONE, 0.2);
        public static final BlinkingLEDState GETTING_CUBE_FLASHING_WHITE =
                new BlinkingLEDState(LEDState.WOULD_EJECT, LEDState.GET_CUBE, 0.2);
        public static final BlinkingLEDState BATTERY_LOW =
                new BlinkingLEDState(LEDState.OFF, LEDState.LOW_BATTERY, 1.0);
        static final BlinkingLEDState LIMELIGHT_MIMICRY = new BlinkingLEDState(LEDState.OFF, LEDState.LIMELIGHT, 1.0);
        public static final BlinkingLEDState IS_DOING_CHARGING_STATION =
                new BlinkingLEDState(LEDState.OFF, LEDState.CHARGING_STATION, 1.0);
        public static final BlinkingLEDState ALIGN_BAD = new BlinkingLEDState(LEDState.OFF, LEDState.ALIGN_BAD, 0.2);

        private final LEDState stateOne = new LEDState(0, 0, 0);
        private final LEDState stateTwo = new LEDState(0, 0, 0);

        private boolean asymmetricDuration;
        private final double duration;
        private final double durationTwo;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            this(stateOne, stateTwo, duration, 0.0);
        }

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double durationOne, double durationTwo) {
            this.stateOne.copyFrom(stateOne);
            this.stateTwo.copyFrom(stateTwo);
            duration = durationOne;
            this.durationTwo = durationTwo;
            asymmetricDuration = durationTwo > 0.0;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if (asymmetricDuration) {
                if (timestamp % (duration + durationTwo) > duration) {
                    desiredState.copyFrom(stateTwo);
                } else {
                    desiredState.copyFrom(stateOne);
                }
            } else {
                if ((int) (timestamp / duration) % 2 == 0) {
                    desiredState.copyFrom(stateOne);
                } else {
                    desiredState.copyFrom(stateTwo);
                }
            }
        }
    }

    class StaticLEDState implements LEDTimedState {
        public static final StaticLEDState staticOff = new StaticLEDState(LEDState.OFF);
        // public static StaticLEDState staticBatteryLow = new StaticLEDState(LEDState.lowBattery);

        LEDState staticState = new LEDState(0, 0, 0);

        public StaticLEDState(LEDState staticState) {
            this.staticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(staticState);
        }
    }
}
