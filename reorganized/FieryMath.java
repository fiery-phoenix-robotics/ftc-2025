package org.firstinspires.ftc.teamcode.FieryMath;

public class FieryMath {
    public static boolean withinRange (double num, double min, double max) {
        return num <= range[0] && num >= range[1];
    }
    public static double angleDifference (double a, double b) {
        return (b - a - 180) % 360 + 180;
    }
}