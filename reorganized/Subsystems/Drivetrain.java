package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external;

import org.firstinspires.ftc.teamcode.FieryMath;

public class Drivetrain extends Subsystem {
    
    public DcMotor leftDriveFront, rightDriveFront, leftDriveRear, rightDriveRear;

    private SparkFunOTOS otis;

    private static Drivetrain instance = null;

    private PIDController xController, yController; //, hController

    public double power;

    public Drivetrain () {
        
    }

    public void init () {
        
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

        configureOtos();

        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        //hController = new PIDController(0, 0, 0);

        drivetrain.setPower(1.0);

    }

    public void setPower(double p) {
        power = p;
    }
    
    public void updateTelemetry () {
        SparkFunOTOS.Pose2D pos = otis.getPosition();
        telemetry.addData("X Position", pos.x);
        telemetry.addData("Y Position", pos.y);
        telemetry.addData("Heading", pos.h);
    }

    public class TeleOp {
        public void doMotion (double cy, double cx, double crx) {
                
            leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            double denominator = Math.max(Math.abs(cy) + Math.abs(cx) + Math.abs(crx), 1);
            double frontLeftPower = (cy + cx + crx) / denominator;
            double backLeftPower = (cy - cx + crx) / denominator;
            double frontRightPower = (cy - cx - crx) / denominator;
            double backRightPower = (cy + cx - crx) / denominator;
            
            leftDriveFront.setPower(frontLeftPower);
            leftDriveRear.setPower(backLeftPower);
            rightDriveFront.setPower(frontRightPower);
            rightDriveRear.setPower(backRightPower);
        }        
    }

    public class Autonomous {
        // returns whether it was successfully able to move to its desired position
        public boolean driveTo (double x, double y) {

            SparkFunOTOS.Pose2D pos = otis.getPosition();
            double current_x = pos.x;
            double current_y = pos.y;
            
            double x_controlled = xController.get(x, current_x);
            double y_controlled = yController.get(y, current_y);
                
            if (opModeIsActive()) {
                leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double northeastPower; // power of northeastwardly-moving motors (front right and back left)
                double northwestPower; // power of northwestwardly-moving motors (front left and back right)
                double e = 4.0; // acceptable error

                // responsively adjust position 
                while (opModeIsActive() && !(FieryMath.withinRange(pos.x, x - e, x + e) && FieryMath.withinRange(pos.y, y - e, y + e))) {
                    pos = otis.getPosition();
                    current_x = pos.x;
                    current_y = pos.y;
                    x_controlled = xController.get(x, current_x);
                    y_controlled = yController.get(y, current_y);

                    double northeast = y_controlled + x_controlled;
                    double northwest = y_controlled - x_controlled;

                    if (Math.abs(northeast) > Math.abs(northwest)) {
                        northeastPower = northeast * (power / Math.abs(northeast));
                        northwestPower = northwest * (power / Math.abs(northeast));
                    } else {
                        northeastPower = northwest * (power / Math.abs(northwest));
                        northwestPower = northeast * (power / Math.abs(northwest));
                    }

                    leftDriveFront.setPower(northwestPower);
                    rightDriveFront.setPower(northeastPower);
                    leftDriveRear.setPower(northeastPower);
                    rightDriveRear.setPower(northwestPower);
                }

                // set motor power back to 0
                leftDriveFront.setPower(0);
                rightDriveFront.setPower(0);
                leftDriveRear.setPower(0);
                rightDriveRear.setPower(0);
            }
        }
        public boolean turnTo (double theta) {
            leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double leftPower;
            double rightPower;
            double e = 4.0; // acceptable error

            // responsively adjust position 
            while (opModeIsActive() && !FieryMath.withinRange(pos.h, h - e, h + e)) {
                pos = otis.getPosition();
                current_h = pos.h;

                double delta_h = -(current_h - theta);

                // accomodate for negative and positive angles with abs. val. greater than 180
                if (Math.abs(delta_h) > 180) {
                    delta_h = Math.signum(delta_h) * (Math.abs(delta_h) - 180);
                }

                leftPower = delta_h / 180;
                rightPower = -delta_h / 180;

                leftDriveFront.setPower(leftPower);
                rightDriveFront.setPower(rightPower);
                leftDriveRear.setPower(leftPower);
                rightDriveRear.setPower(rightPower);
            }

            // set motor power back to 0
            leftDriveFront.setPower(0);
            rightDriveFront.setPower(0);
            leftDriveRear.setPower(0);
            rightDriveRear.setPower(0);
        }
        private void configureOtos() {

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
    }

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
}