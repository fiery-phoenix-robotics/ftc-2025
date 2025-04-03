package org.firstinspires.ftc.lib.control;

import org.firstinspires.ftc.lib.arithmetic.Arithmetic;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    public ElapsedTime timer = new ElapsedTime();

    public ElapsedTime stallTimer = new ElapsedTime();

    // maximum time that the motor can stall before the power should be cut
    private double stallingTimeout = 1.0;

    private double kP, kI, kD;

    private double maxIntegral; 
    
    private double integral = 0, lastError = 0, lastTarget = 0;

    public PIDController (double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        maxIntegral = kI * 0.25;
    }

    public double calculateError(double t, double i) {
        return t - i;
    }

    public double get (double target, double initial) {
        double ret = 0;
        double error = calculateError(target, initial);
        double derivative = (error - lastError) / timer.seconds();
        integral += error * timer.seconds();

        if (!Arithmetic.withinRange(integral, -maxIntegral, maxIntegral))
            integral = Math.signum(integral) * maxIntegral;

        if (lastTarget != target)
            integral = 0;

        if (derivative > 0.0 || stallTimer.seconds() < stallingTimeout)
            ret = kP * error + kI * integral + kD * derivative;

        if (derivative > 0.0)
            stallTimer.reset();

        lastError = error;
        lastTarget = target;

        timer.reset();
        
        return ret;
    }

}