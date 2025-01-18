package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "MecanumAutoJava_FINAL_PUSH", group="Competition", preselectTeleOp="StarterBot2025_FINAL_2DRIVER")
public class MecanumAutoJava_FINAL_PUSH extends LinearOpMode {

  private DcMotor leftDriveFront;
  private DcMotor rightDriveFront;
  private DcMotor leftDriveRear;
  private DcMotor rightDriveRear;
  private DcMotor arm;
  private CRServo intake;
  private Servo claw;
    
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 10; // 550
    private static final int ARM_POSITION_LOW_BASKET = 2265;
    private static final int ARM_POSITION_SPECIMEN_HIGH = 2580;
    private static final int ARM_POSITION_SPECIMEN_WALL = 1227;
    private static final int ARM_POSITION_HANG_L2 = 2500;

  
  //Convert from the counts per revolution of the encoder to counts per inch
  static final double HD_COUNTS_PER_REV = 28;
  static final double DRIVE_GEAR_REDUCTION = 20.15293;
  static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
  static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
  static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
  
  //Create elapsed time variable and an instance of elapsed time
  private ElapsedTime runtime = new ElapsedTime();
  
    // drive function that drives to a certain polar coordinate relative to the current position
    private void drive(double power, double r, double theta) {
    int frontLeftTarget;
    int frontRightTarget;
    int rearLeftTarget;
    int rearRightTarget;
    
    // convert theta to radians so java trig functions work
    double thetaAsRadians = Math.toRadians(theta);
    
    if (opModeIsActive()) {
     // Create target positions, use trig to calculate factor to multiply by
      frontLeftTarget = leftDriveFront.getCurrentPosition() + (int)(1.3 * r * (Math.sin(thetaAsRadians) - Math.cos(thetaAsRadians)) * DRIVE_COUNTS_PER_IN);
      frontRightTarget = rightDriveFront.getCurrentPosition() - (int)(1.3 * r * (Math.sin(thetaAsRadians) + Math.cos(thetaAsRadians)) * DRIVE_COUNTS_PER_IN);
      rearLeftTarget = leftDriveRear.getCurrentPosition() + (int)(r * (Math.sin(thetaAsRadians) + Math.cos(thetaAsRadians)) * DRIVE_COUNTS_PER_IN);
      rearRightTarget = rightDriveRear.getCurrentPosition() - (int)(r * (Math.sin(thetaAsRadians) - Math.cos(thetaAsRadians)) * DRIVE_COUNTS_PER_IN);
      
      // set target positions
      leftDriveFront.setTargetPosition(frontLeftTarget);
      rightDriveFront.setTargetPosition(frontRightTarget);
      leftDriveRear.setTargetPosition(rearLeftTarget);
      rightDriveRear.setTargetPosition(rearRightTarget);

      //switch to run to position mode
      leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      //run to position at the desiginated power
      leftDriveFront.setPower(power);
      rightDriveFront.setPower(power);
      leftDriveRear.setPower(power);
      rightDriveRear.setPower(power);
      
      double e = 0.25;
      // wait until both motors are no longer busy running to position
      while (opModeIsActive() && (leftDriveFront.isBusy() || rightDriveFront.isBusy() || leftDriveRear.isBusy() || rightDriveRear.isBusy())) {
          
      }
      
      // set motor power back to 0
      leftDriveFront.setPower(0);
      rightDriveFront.setPower(0);
      leftDriveRear.setPower(0);
      rightDriveRear.setPower(0);
    }
}
  
    // turn function that turns to a certain angle
    private void turn(double power, double theta) {
    int frontLeftTarget;
    int frontRightTarget;
    int rearLeftTarget;
    int rearRightTarget;
    
    // convert angle to rotation distance (18 * sqrt2 = diagonal of robot)
    double arcLength = (2 * Math.PI *  (18 * Math.sqrt(2))/2) * (theta / 360);
    
    if (opModeIsActive()) {
     // Create target positions by multiplying arc length (distance to move to rotate a certain amount) by a conversion factor
      frontLeftTarget = leftDriveFront.getCurrentPosition() + (int)(arcLength * DRIVE_COUNTS_PER_IN);
      frontRightTarget = rightDriveFront.getCurrentPosition() + (int)(arcLength * DRIVE_COUNTS_PER_IN);
      rearLeftTarget = leftDriveRear.getCurrentPosition() + (int)(arcLength * DRIVE_COUNTS_PER_IN);
      rearRightTarget = rightDriveRear.getCurrentPosition() + (int)(arcLength * DRIVE_COUNTS_PER_IN);
      
      // set target positions
      leftDriveFront.setTargetPosition(frontLeftTarget);
      rightDriveFront.setTargetPosition(frontRightTarget);
      leftDriveRear.setTargetPosition(rearLeftTarget);
      rightDriveRear.setTargetPosition(rearRightTarget);

      //switch to run to position mode
      leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      //run to position at the desiginated power
      leftDriveFront.setPower(power);
      rightDriveFront.setPower(power);
      leftDriveRear.setPower(power);
      rightDriveRear.setPower(power);
      
      // wait until both motors are no longer busy running to position
      while (opModeIsActive() && (leftDriveFront.isBusy() || rightDriveFront.isBusy() || leftDriveRear.isBusy() || rightDriveRear.isBusy())) {
          
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

    
    // 10-point cycle: drive to submersible and back and score specimen
    private void firstCycle(double power) {
        drive(power, 18, 270);
        drive(power, 12, 90);
    }
    
    // 10-point cycle: drive to wall and back, then score specimen on high bar
    private void middleCycle(double power) {
        // go to observation zone and back
        drive(power, 10, 90);
        drive(power, 60, 0);
        turn(power, 180);
        drive(power, 10, 90);
        moveArmTo(power, ARM_POSITION_SPECIMEN_WALL);
        moveClawTo(1.3);
        drive(power, 10, 270);
        turn(power, 180);
        drive(power, 60, 180);
        drive(power, 10, 270);
        moveArmTo(power, ARM_POSITION_SPECIMEN_HIGH);
        drive(power, 27, 90);
        moveArmTo(power, 1900);
        moveClawTo(0.5);
        releaseArm();
        drive(power, 27, 270);
    }
    
    // 3-point cycle: drive to ascent zone
    private void finalCycle(double power) {
        drive(power, 8, 90);
        drive(power, 108, 0);
        releaseArm();
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
        
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        if (opModeIsActive()) {
          firstCycle(1);
        }
    }
}
