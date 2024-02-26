package frc.robot;

public final class Constants {
    public static class Mapping {
        public static class Claw {
            public static int intakeMotor = 1; // change to correct ids later
        }
    }

    public static class Generic {
        public static int timeoutMs = 3000;
    }

    public static class Config {
        public static class Drive {
            public static class Power {
                public static double kOpenLoopRamp = 0.0;
            }
        }
    }
}
