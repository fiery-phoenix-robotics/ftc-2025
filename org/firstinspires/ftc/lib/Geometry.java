package org.firstinspires.ftc.lib;

public class Geometry {
    public static double angleDifference (double a, double b) {
        return (b - a - 180) % 360 + 180;
    }
}
