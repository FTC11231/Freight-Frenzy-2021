package org.firstinspires.ftc.teamcode.PreSeason.Utils;

public class MathUtil {

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
