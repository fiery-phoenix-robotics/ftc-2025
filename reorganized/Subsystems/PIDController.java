package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    public ElapsedTime timer = new ElapsedTime();

    private double kP, kI, kD;

    private double maxIntegral; 
    
    private double integral = 0, lastError = 0, lastTarget = 0;

    public PIDController (double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        maxIntegral = kI * 0.25;
    }

    public double get (double target, double initial) {
        double error = target - initial;
        double derivative = (error - lastError) / timer.seconds();
        integral += error * timer.seconds();

        if (!FieryMath.withinRange(integral, -maxIntegral, maxIntegral))
            integral = Math.signum(integral) * maxIntegral;

        if (lastTarget != target)
            integral = 0;

        lastError = error;
        lastTarget = target;

        timer.reset();

        return kP * error + kI * integral + kD * derivative;
    }

}