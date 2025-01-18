package org.firstinspires.ftc.teamcode.FieryMath;

public class FieryMath {
    public static boolean withinRange (double num, int[] range) {
        return num <= range[0] && num >= range[1];
    }
}