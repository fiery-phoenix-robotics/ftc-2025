package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.FieryMath;

public class Drivetrain extends Subsystem {
    
    public DcMotor leftDriveFront, rightDriveFront, leftDriveRear, rightDriveRear;

    private SparkFunOTOS otis;

    private static Drivetrain instance = null;

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

    }

    public class TeleOp {
        public void doMotion (double c_y, double c_x, double c_rx) {
                
            leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            double denominator = Math.max(Math.abs(c_y) + Math.abs(c_x) + Math.abs(c_rx), 1);
            double frontLeftPower = (c_y + c_x + c_rx) / denominator;
            double backLeftPower = (c_y - c_x + c_rx) / denominator;
            double frontRightPower = (c_y - c_x - c_rx) / denominator;
            double backRightPower = (c_y + c_x - c_rx) / denominator;
            
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
            
            double delta_x = x - current_x;
            double delta_y = y - current_y;

                
            if (opModeIsActive()) {
                leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double northeastPower; // power of northeastwardly-moving motors (front right and back left)
                double northwestPower; // power of northwestwardly-moving motors (front left and back right)
                double e = 4; // acceptable error

                // responsively adjust position 
                while (opModeIsActive() && !(FieryMath.withinRange(pos.x, {x - e, x + e}) && FieryMath.withinRange(pos.y, {y - e, y + e}))) {
                    pos = otis.getPosition();
                    current_x = pos.x;
                    current_y = pos.y;

                    delta_x = x - current_x;
                    delta_y = y - current_y;

                    double northeast = delta_y + delta_x;
                    double northwest = delta_y - delta_x;

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