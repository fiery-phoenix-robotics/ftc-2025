package org.firstinspires.ftc.lib.geometry;

import java.awt.geom.Point2D;

public class Pose2d {

    private Point2D position;
    private double heading;

    public Pose2d (double x, double y, double h) {
        position = new Point2D(x, y);
        heading = h;
    }
    
    public String toString () {
        return "(" + x + ", " + y + ", " + rz + ")";
    }

    public Point2D getPosition () {
        return position;
    }

    public double getX () {
        return position.getX();
    }

    public double getY () {
        return position.getY();
    }

    public double getHeading () {
        return heading;
    }

}
