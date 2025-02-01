package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.FieryMath;

public class AngularPIDController {

    public AngularPIDController (double p, double i, double d) {
        super(p, i, d);
    }

    @Override
    public double calculateError(double t, double i) {
        return FieryMath.angleDifference(i, t);
    }

}