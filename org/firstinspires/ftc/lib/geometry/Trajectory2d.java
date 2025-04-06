package org.firstinspires.ftc.lib.geometry;

import javax.vecmath.Vector2d;

// a trajectory in in/s.
public class Trajectory2d {

    private Vector2d linear;
    private double rz;

    public Trajectory2d (double x, double y, double rz) {
        this.linear = new Vector2d(x, y);
        linear.normalize();
        this.rz = rz;
    }

    public String toString () {
        return "(" + x + ", " + y + ", " + rz + ")";
    }

    public Vector2d getLinear () {
        return linear;
    }

    public double getX () {
        return linear.getX();
    }

    public double getY () {
        return linear.getY();
    }

    public double getRZ () {
        return rz;
    }
    
}