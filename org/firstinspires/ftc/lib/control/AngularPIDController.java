package org.firstinspires.ftc.lib.control;

import org.firstinspires.ftc.lib.geometry.Geometry;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AngularPIDController extends PIDController {

    public AngularPIDController (double p, double i, double d) {
        super(p, i, d);
    }

    @Override
    public double calculateError(double t, double i) {
        return Geometry.angleDifference(i, t);
    }

}