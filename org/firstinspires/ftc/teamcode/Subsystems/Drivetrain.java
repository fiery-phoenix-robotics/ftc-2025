package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.lib.geometry.Trajectory2d;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.lib.control.PIDController;
import org.firstinspires.ftc.lib.geometry.Pose2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Drivetrain extends Subsystem {

    private static Drivetrain instance;
    
    private DcMotor leftDriveFront, rightDriveFront, leftDriveRear, rightDriveRear;
    private PIDController leftFrontController, rightFrontController, leftRearController, rightRearController;

    private SparkFunOTOS otis;

    private Pose2d pose;

    public Vision vision;

    private Drivetrain () {

    }

    public void init () {

        drivetrain.leftDriveFront = hardwareMap.get(DcMotor.class, "leftDriveFront");
        drivetrain.rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        drivetrain.leftDriveRear = hardwareMap.get(DcMotor.class, "leftDriveRear");
        drivetrain.rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        drivetrain.otis = hardwareMap.get(SparkFunOTOS.class, "otis");

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);
        
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontController = new PIDController(Constants.Drivetrain.LeftFront.kP, Constants.Drivetrain.LeftFront.kI, Constants.Drivetrain.LeftFront.kD);
        rightFrontController = new PIDController(Constants.Drivetrain.RightFront.kP, Constants.Drivetrain.RightFront.kI, Constants.Drivetrain.RightFront.kD);
        leftRearController = new PIDController(Constants.Drivetrain.LeftRear.kP, Constants.Drivetrain.LeftRear.kI, Constants.Drivetrain.LeftRear.kD);
        rightRearController = new PIDController(Constants.Drivetrain.RightRear.kP, Constants.Drivetrain.RightRear.kI, Constants.Drivetrain.RightRear.kD);

        this.configureOtos();

        vision = Vision.getInstance();
        pose = vision.findPose();

    }

    public void loop () {

        SparkFunOTOS.Pose2D pos = otis.getPosition();
        pose = new Pose2d(pose.getX() + pos.x, pose.getY + pos.y, pose.getHeading() + pos.h);

    }

    public void drive (Trajectory2d t) {

        double x = t.getX();
        double y = t.getY();
        double h = t.getRZ();

        double frontLeftSpeed = (y + x + h) * Constants.Drivetrain.maxSpeeed;
        double backLeftSpeed = (y - x + h) * Constants.Drivetrain.maxSpeeed;
        double frontRightSpeed = (y - x - h) * Constants.Drivetrain.maxSpeeed;
        double backRightSpeed = (y + x - h) * Constants.Drivetrain.maxSpeeed;

        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double frontLeftPower = leftFrontController.get(frontLeftSpeed, 0);
        double backLeftPower = leftRearController.get(backLeftSpeed, 0);
        double frontRightPower = rightFrontController.get(frontRightSpeed, 0);
        double backRightPower = rightRearController.get(backRightSpeed, 0);
        
        leftDriveFront.setPower(frontLeftPower);
        leftDriveRear.setPower(backLeftPower);
        rightDriveFront.setPower(frontRightPower);
        rightDriveRear.setPower(backRightPower);

        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Rear Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Rear Right Power", backRightPower);

    }

    private void configureOtos () {

        otis.setLinearUnit(DistanceUnit.INCH);
        otis.setAngularUnit(AngleUnit.DEGREES);
        // define how far the sensor is offset from the tracking point of the robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otis.setOffset(offset);
        otis.setLinearScalar(1.0);
        otis.setAngularScalar(1.0);
        otis.calibrateImu();
        otis.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otis.setPosition(currentPosition);

    }

    public static Drivetrain getInstance() {

        if (instance == null) {
            instance = new Drivetrain();
        }

        return instance;

    }

}
