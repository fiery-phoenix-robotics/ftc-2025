package org.firstinspires.ftc.lib.geometry;

// a trajectory in in/s.
public class Trajectory {
    public double x, y, rz;
    public Trajectory (double x, double y, double rz) {
        this.x = x;
        this.y = y;
        this.rz = rz;
    }
    public String toString () {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}