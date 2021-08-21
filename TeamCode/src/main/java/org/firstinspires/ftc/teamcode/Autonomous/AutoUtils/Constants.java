package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;

public class Constants {

    public static final double ticksPerRev = 28;

    public static class Drivetrain {
        public static final double gearRatio = 20;
        public static final double wheelDiameter = 4;

        public static final double ticksPerInch = -ticksPerRev * gearRatio / (wheelDiameter * Math.PI);

        public static final double turningPIDkF = 0.18;
        public static final double turningPIDkP = 0.008;
        public static final double turningPIDkI = 0;
        public static final double turningPIDkD = 0.0009;
    }
}
