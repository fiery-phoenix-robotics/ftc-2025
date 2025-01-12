package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "OTOSMecanumAutoJava_12282024", group="Competition", preselectTeleOp="StarterBot2025_FINAL_2DRIVER")
public class OTOSMecanumAutoJava_12282024 extends LinearOpMode {

  private DcMotor leftDriveFront;
  private DcMotor rightDriveFront;
  private DcMotor leftDriveRear;
  private DcMotor rightDriveRear;
  private DcMotor arm;
  private CRServo intake;
  private Servo claw;
  private SparkFunOTOS otis;
    
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 10; // 550
    private static final int ARM_POSITION_LOW_BASKET = 2265;
    private static final int ARM_POSITION_SPECIMEN_HIGH = 2580;
    private static final int ARM_POSITION_SPECIMEN_WALL = 1227;
    private static final int ARM_POSITION_HANG_L2 = 2500;

  
  //Create elapsed time variable and an instance of elapsed time
  private ElapsedTime runtime = new ElapsedTime();
  
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
  
    // drive function that sends robot to a certain field position (x, y) in inches
    private void drive(double power, double x, double y) {
    
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
      double e = 0.25; // acceptable error
      
      // responsively adjust position 
      while (opModeIsActive() && (!(x - e < current_x && current_x < x + e) || !(y - e < current_y && current_y < y + e))) {
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

    // turn function that turns a robot to a certain angle theta in degrees
    private void turn(double power, double theta) {
        SparkFunOTOS.Pose2D pos = otis.getPosition();
        double current_h = pos.h;
        
        double delta_h = theta - current_h;
        
            if (opModeIsActive()) {

      leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      
      double clockwisePower; // power of clockwise-moving motors (left)
      double counterclockwisePower; // power of counterclockwise-moving motors (right)
      double e = 0.25; // acceptable error in degrees
      
      // responsively adjust position 
      while (opModeIsActive() && !(current_h < theta - e && theta + e < current_h)) {
        pos = otis.getPosition();
        current_h = pos.h;
        
        delta_h = theta - current_h;
        
        clockwisePower = -delta_h / 360;
        counterclockwisePower = delta_h / 360;
        
        leftDriveFront.setPower(clockwisePower);
        rightDriveFront.setPower(counterclockwisePower);
        leftDriveRear.setPower(clockwisePower);
        rightDriveRear.setPower(counterclockwisePower);
      }
      
      
      
      // set motor power back to 0
      leftDriveFront.setPower(0);
      rightDriveFront.setPower(0);
      leftDriveRear.setPower(0);
      rightDriveRear.setPower(0);
    }

    }
    
    // function to run arm to targetpositon
    private void moveArmTo(double power, int target) {
      arm.setTargetPosition(target);

      //switch to run to position mode
      arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      //run to position at the desiginated power
      arm.setPower(power);
      
      // wait until both motors are no longer busy running to position
      while (opModeIsActive() && (arm.isBusy())) {
          
      }
      
      //
          }
    
    private void releaseArm() {
        arm.setPower(0);
    }
    
    private void moveClawTo(double target) {

      claw.setPosition(target);
      
      try {
        Thread.sleep(1000);
      }
      catch (InterruptedException e) {
        
      }
    }
    
    @Override
    public void runOpMode() {
    
        leftDriveFront = hardwareMap.get(DcMotor.class, "leftDriveFront");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        leftDriveRear = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        otis = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        configureOtos();
        
        waitForStart();
        if (opModeIsActive()) {
            // the magic happens here!
        }
    }
}