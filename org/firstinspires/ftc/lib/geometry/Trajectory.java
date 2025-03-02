package org.firstinspires.ftc.lib.geometry;

// a trajectory in in/s.
public class Trajectory {
    private double x;
    private double y;
    private double rz;
    public Trajectory (double x, double y, double rz) {
        this.x = x;
        this.y = y;
        this.rz = rz;
    }
    public String toString () {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}