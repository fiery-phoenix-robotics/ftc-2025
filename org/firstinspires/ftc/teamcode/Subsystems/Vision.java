package org.firstinspires.ftc.teamcode.Subsystems;

public class Vision extends Subsystem {
    
    private static Vision instance;

    private Vision () {

    }

    public Pose2d findPose () {
        Pose2d pose = new Pose2d();
        // limelight stuff here...
        return pose;
    }

    public static Vision getInstance () {

        if (instance == null) {
            instance = new Vision();
        }

        return instance;

    }

}
