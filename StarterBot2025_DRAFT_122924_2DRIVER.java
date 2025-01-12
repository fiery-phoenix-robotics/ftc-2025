package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Map;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="StarterBot2025_DRAFT_122924_2DRIVER", group="Competition")
public class StarterBot2025_DRAFT_122924_2DRIVER extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveRear = null;
    private DcMotor rightDriveRear = null;
    private DcMotor arm = null;
    private DcMotor wrist = null;
    private Servo claw = null;
    private CRServo intake = null;
    
    private static final int[] armRange = {0, 3765};
    private static final int[] wristRange = {-20, 600};
    
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE_FLOOR = 10; // 550
    private static final int ARM_POSITION_LOW_BASKET = 2265;
    private static final int ARM_POSITION_HIGH_BASKET = 3765;
    private static final int ARM_POSITION_SPECIMEN_HIGH = 2610;
    private static final int ARM_POSITION_SPECIMEN_WALL = 1227;
    private static final int ARM_POSITION_HANG_L2 = 2500;
    private static final int ARM_POSITION_INTAKE_SUBMERSIBLE = 1120;

    private static final int WRIST_POSITION_INIT = -18;
    private static final int WRIST_POSITION_INTAKE_FLOOR =  350;//=425
    private static final int WRIST_POSITION_LOW_BASKET = 415;
    private static final int WRIST_POSITION_HIGH_BASKET = 585;
    private static final int WRIST_POSITION_SPECIMEN_HIGH = 0;
    private static final int WRIST_POSITION_SPECIMEN_WALL = 0;
    private static final int WRIST_POSITION_HANG_L2 = 0;
    private static final int WRIST_POSITION_INTAKE_SUBMERSIBLE = 600;
    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.5;
    private static final double CLAW_CLOSED_POSITION = 1.0;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE_FLOOR,
        INTAKE_SUBMERSIBLE,
        LOW_BASKET,
        HIGH_BASKET,
        SPECIMEN_HIGH,
        SPECIMEN_WALL,
        HANG_L2,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = false;
    private boolean lastBump = false;
    private boolean runIntake = false;

    //target position
    private int targetArm = 0;
    private int targetWrist = 0;
    private int lastTargetArm = targetArm;
    private int lastTargetWrist = targetWrist;
    
    //power variables
    private double armPower = 1.0;
    private double wristPower = 1.0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Stop and reset encoders
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set motor direction
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);
        //Set zero power behavior
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //initialize gripper, wrist, and arm
        boolean clawInitialized = false;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        targetArm = ARM_POSITION_INIT;
        targetWrist = WRIST_POSITION_INIT;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Handle state transitions based on gamepad input
            
            //Button pad controls
            if (gamepad1.a) {
                currentState = RobotState.INTAKE_FLOOR;
                runIntake = true;
            } else if (gamepad1.b) {
                claw.setPosition(CLAW_CLOSED_POSITION);
                currentState = RobotState.INIT;
                runIntake = false;
            } else if (gamepad1.x) { 
                currentState = RobotState.HIGH_BASKET;
                runIntake = false;
            } else if (gamepad1.y) {
                currentState = RobotState.SPECIMEN_HIGH;
                runIntake = false;
            } else if (gamepad1.back) {
                currentState = RobotState.SPECIMEN_WALL;
                runIntake = false;
            } else if (gamepad1.dpad_up){ //manual control
                currentState = RobotState.MANUAL;
                targetArm += 20;
                runIntake = false;
            } else if (gamepad1.dpad_down){
                currentState = RobotState.MANUAL;
                targetArm -= 20;
                runIntake = false;
            } else if (gamepad1.dpad_left){
                currentState = RobotState.MANUAL;
                targetWrist -= 10;
                runIntake = false;
            } else if (gamepad1.dpad_right){
                currentState = RobotState.MANUAL;
                targetWrist += 10;
                runIntake = false;
            } else if (gamepad1.start){
                currentState = RobotState.HANG_L2;
                runIntake = false;
            } else if (gamepad1.left_bumper) {
                currentState = RobotState.INTAKE_SUBMERSIBLE;
                runIntake = true;
            } else {
                currentState = RobotState.MANUAL;
            }
            
            // State machine logic
            switch (currentState) {
                case INIT:
                    //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    targetArm = ARM_POSITION_INIT;
                    targetWrist = WRIST_POSITION_INIT;
                    //wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addData("State", "INIT");
                    break;
                case INTAKE_FLOOR:
                    targetArm = ARM_POSITION_INTAKE_FLOOR;
                    targetWrist = WRIST_POSITION_INTAKE_FLOOR;
                    telemetry.addData("State", "INTAKE_FLOOR");
                    break;
                case INTAKE_SUBMERSIBLE:
                    targetArm = ARM_POSITION_INTAKE_SUBMERSIBLE;
                    targetWrist = WRIST_POSITION_INTAKE_SUBMERSIBLE;
                    telemetry.addData("State", "INTAKE_SUBMERSIBLE");
                    break;
                case LOW_BASKET:
                    targetArm = ARM_POSITION_LOW_BASKET;
                    targetWrist = WRIST_POSITION_LOW_BASKET;
                    telemetry.addData("State", "LOW_BASKET");
                    break;
                case HIGH_BASKET:
                    targetArm = ARM_POSITION_HIGH_BASKET;
                    targetWrist = WRIST_POSITION_HIGH_BASKET;
                    telemetry.addData("State", "HIGH_BASKET");
                    break;
                case SPECIMEN_HIGH:
                    targetArm = ARM_POSITION_SPECIMEN_HIGH;
                    targetWrist = WRIST_POSITION_SPECIMEN_HIGH;
                    telemetry.addData("State", "SPECIMEN_HIGH");
                    break;
                case SPECIMEN_WALL:
                    targetArm = ARM_POSITION_SPECIMEN_WALL;
                    targetWrist = WRIST_POSITION_SPECIMEN_WALL;
                    telemetry.addData("State", "SPECIMEN_WALL");
                    break;                                                              
                case HANG_L2:
                    targetArm = ARM_POSITION_HANG_L2;
                    targetWrist = WRIST_POSITION_HANG_L2;
                    telemetry.addData("State", "HANG_L2");
                    break;
                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }
            
            // SET Power to slow for Intake
            if (currentState == RobotState.INIT) {
                armPower = 0.5;
                wristPower = 0.5;
            } else {
                armPower = 1.0;
                wristPower = 1.0;
            }

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBump) {
            //if (gamepad2.right_bumper) {
                if (!clawInitialized) {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                    clawInitialized = true;
                }
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            // Control intake servo with triggerss
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(1.0) ;
            } else if (gamepad1.left_trigger>0.1) {
                intake.setPower(-3.0);//-1.0);
            } else if (runIntake) {
                intake.setPower(1.0) ;
            } else {
                intake.setPower(0);
            }
            
            //DRIVE Split Arcade
             double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftDriveFront.setPower(frontLeftPower);
            leftDriveRear.setPower(backLeftPower);
            rightDriveFront.setPower(frontRightPower);
            rightDriveRear.setPower(backRightPower);
            
            //double drive = -gamepad1.left_stick_y;
           //double turn  =  -gamepad1.right_stick_x;
            //double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
           // double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
    
          //  leftDrive.setPower(leftPower);
         //   rightDrive.setPower(rightPower);
            
            //Only move the arm or wrist if one of the arm or wrist modes is active
            //if ((targetArm != lastTargetArm) || (targetWrist != lastTargetWrist)) {
                if (wristRange[0] <= targetWrist && targetWrist <= wristRange[1]) { //change this!
                    wrist.setTargetPosition(targetWrist);
                }
                else if (wristRange[0] > targetWrist) {
                    wrist.setTargetPosition(wristRange[0]);
                }
                else {
                    wrist.setTargetPosition(wristRange[1]);
                }
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(wristPower); 
                
                if (runIntake) {
                    //Move wrist first, then arm to account for submersible bar
                    //Timeout if wrist stays busy too long
                    long currentTime = System.currentTimeMillis();
                    long endTime = currentTime + 1500;
                    
                    while (wrist.isBusy() && (currentTime < endTime)) {
                        currentTime = System.currentTimeMillis();
                    }
                }
                
                if (armRange[0] <= targetArm && targetArm <= armRange[1]) { //change this!
                    arm.setTargetPosition(targetArm);
                }
                else if (armRange[0] > targetArm) {
                    arm.setTargetPosition(armRange[0]);
                }
                else {
                    arm.setTargetPosition(armRange[1]);
                }
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                
                lastTargetArm = targetArm;
                lastTargetWrist = targetWrist;
                telemetry.addData("MOVE ARM OR WRIST", "YES");
            //}

            // Send telemetry data to the driver station
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Claw Position Double", claw.getPosition());
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Wrist Power", wrist.getPower());
            telemetry.addData("Last Target Arm", lastTargetArm);
            telemetry.addData("Last Target Wrist", lastTargetWrist);
            telemetry.addData("Target Arm", targetArm);
            telemetry.addData("Target Wrist", targetWrist);
            telemetry.update();
        }
    }
}
