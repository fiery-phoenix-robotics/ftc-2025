package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Map;
import java.util.concurrent.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ThreadedTeleOp_DRAFT", group="Competition")
public class ThreadedTeleOp extends LinearOpMode {
    
    private static DcMotor leftDriveFront = null;
    private static DcMotor rightDriveFront = null;
    private static DcMotor leftDriveRear = null;
    private static DcMotor rightDriveRear = null;
    private static DcMotor arm = null;
    private static DcMotor wrist = null;
    private static CRServo intake = null;
    private static DcMotor lever = null;
    private static Servo claw = null;
    private static SparkFunOTOS otis = null;
    
    public static int targetArm = 0;
    public static int targetWrist = 0;
    public static int targetLever = 0;
    
    private static double armPower = 1.0;
    private static double wristPower = 1.0;
    private static double leverPower = 1.0;
    
    public static int armMarginOfError = 10;
    public static int wristMarginOfError = 2;
    public static int leverMarginOfError = 2;
    
    private static boolean clawOpen = true;
    private static boolean clawAlreadyToggled = false;
    
    public static double x;
    public static double y;
    public static double rx;
    
    public static int moveArmBy;
    public static int moveWristBy;
    
    public static int towerActionTimeout = 3000;
    
    public enum TowerAction implements Runnable {
        // INIT, INTAKE_FLOOR, SCORE_HIGH_BASKET, SCORE_SPECIMEN_HIGH, GRAB_SPECIMEN_WALL,
        // MANUAL_RAISE_ARM, MANUAL_LOWER_ARM, MANUAL_EXTEND_WRIST, MANUAL_RETRACT_WRIST
        // MANUAL_INTAKE, MANUAL_OUTTAKE, TOGGLE_CLAW
        INIT {
            public void run() {
                    moveWristTo(0, 0.5);
                    moveArmTo(0, 0.5);
                    moveLeverTo(0, 0.5);
            }
        },
        GO_TO_INTAKE {
            public void run () {
                    moveArmTo(0);
                    moveWristTo(980);
                    intake.setPower(-1.0);
                    try {
                        Thread.sleep(3000);
                    }
                    catch (InterruptedException e) {
                        
                    }
                    intake.setPower(0.0);
            }
        },
        SCORE_HIGH_BASKET {
            public void run () {
                    moveWristTo(200);
                    moveArmTo(3690);
                    moveWristTo(1800);
                    try {
                        Thread.sleep(3000);
                    }
                    catch (InterruptedException e) {
                        
                    }
                    intake.setPower(1.0);
                    try {
                        Thread.sleep(3000);
                    }
                    catch (InterruptedException e) {
                        
                    }
                    intake.setPower(0.0);
            }
        },
        SCORE_SPECIMEN_HIGH {
            public void run() {
                    moveWristTo(0);
                    moveLeverTo(290, 1.0);
            }
        },
        GRAB_SPECIMEN_WALL {
            public void run() {
                setClaw(true);
                moveLeverTo(45);
                try {
                    Thread.sleep(3000);
                }
                catch (InterruptedException e) {
                    
                }
                setClaw(false);
            }
        },
        BEFORE_SPECIMEN_HIGH {
            public void run() {
                moveLeverTo(465);
            }
        },
        ASCEND_LEVEL_TWO {
            public void run() {
                    moveWristTo(0);
                    moveArmTo(2500);
                    try {
                        Thread.sleep(2000);
                    }
                    catch (InterruptedException e) {
                        
                    }
                    moveArmTo(0);
            }
        },
        MANUAL_MOVE_ARM {
            public void run() {
                if (0 < arm.getCurrentPosition() + moveArmBy && arm.getCurrentPosition() + moveArmBy < 36765) {
                    targetArm += moveArmBy;
                } else if (arm.getCurrentPosition() < 0){
                    targetArm = 0;
                } else if (arm.getCurrentPosition() > 3765) {
                    targetArm = 3765;
                } else {
                    return;
                }
                armPower = 1.0;
                
                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1.0);
            }
        },
        MANUAL_RAISE_LEVER {
            public void run() {
                targetLever += 2;
                leverPower = 1.0;
                
                lever.setTargetPosition(targetLever);
                lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lever.setPower(leverPower);
            }
        },
        MANUAL_LOWER_LEVER {
            public void run() {
                targetLever -= 2;
                leverPower = 0.5;
                
                lever.setTargetPosition(targetLever);
                lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lever.setPower(leverPower);
            }
        },
        MANUAL_MOVE_WRIST {
            public void run() {
                if (0 < wrist.getCurrentPosition() + moveWristBy && wrist.getCurrentPosition() + moveWristBy < 2000) {
                    targetWrist += moveWristBy;
                } else if (wrist.getCurrentPosition() < 0){
                    targetWrist = 0;
                } else if (wrist.getCurrentPosition() > 2000) {
                    targetWrist = 2000;
                } else {
                    return;
                }
                wristPower = 1.0;
                
                wrist.setTargetPosition(targetWrist);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(1.0);
            }
        },
        MANUAL_INTAKE {
            public void run() {
                    intake.setPower(-1.0);
            }
        },
        MANUAL_OUTTAKE {
            public void run() {
                    intake.setPower(1.0);
            }
        },
        IDLE {
            public void run () {
                    intake.setPower(0);
            }
        };
    }
    
    public enum DrivetrainAction implements Runnable {
        DRIVE {
            public void run () {
                
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
            }
        },
        IDLE {
            public void run () {
                leftDriveFront.setPower(0);
                leftDriveRear.setPower(0);
                rightDriveFront.setPower(0);
                rightDriveRear.setPower(0);
            }
        }
    }
    
    public enum ClawAction implements Runnable {
        TOGGLE {
            public void run() {
                toggleClaw();
            }
        },
        OPEN {
            public void run() {
                setClaw(true);
            }
        },
        CLOSE {
            public void run() {
                setClaw(false);
            }
        },
        IDLE {
            public void run() {
                try {
                    Thread.sleep(1);
                }
                catch (InterruptedException e) {
                    
                }
            }
        }
    }
    
    private TowerAction lastTowerAction = TowerAction.INIT;
    private DrivetrainAction lastDrivetrainAction = DrivetrainAction.IDLE;
    private ClawAction lastClawAction = ClawAction.CLOSE;
        
    Thread towerThread = new Thread(lastTowerAction);
    Thread drivetrainThread = new Thread(lastDrivetrainAction);
    Thread clawThread = new Thread(lastClawAction);
    
    @Override
    public void runOpMode () {
        // Initialize the hardware variables.
        leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        lever = hardwareMap.get(DcMotor.class, "lever");
        claw = hardwareMap.get(Servo.class, "claw");
        otis = hardwareMap.get(SparkFunOTOS.class, "otis");

        // Stop and reset encoders
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set motor direction
        lever.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);
        
        //Set zero power behavior
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        configureOtos();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        towerThread = new Thread(lastTowerAction);
        drivetrainThread = new Thread(lastDrivetrainAction);
        clawThread = new Thread(lastClawAction);

        towerThread.start();
        drivetrainThread.start();
        clawThread.start();
        
        String lastButtonPressed = "";
        String thisButtonPressed = "";
        
        while (opModeIsActive()) {
            
            if (gamepad1.a) {
                thisButtonPressed = "a";
            }
            else if (gamepad1.b) {
                thisButtonPressed = "b";
            }
            else if (gamepad1.x) {
                thisButtonPressed = "x";
            }
            else if (gamepad1.y) {
                thisButtonPressed = "y";
            }
            else if (gamepad1.back) {
                thisButtonPressed = "back";
            }
            else if (gamepad1.left_stick_y != 0.0) {
                thisButtonPressed = "left_stick_y";
            }
            else if (gamepad1.right_stick_y != 0.0) {
                thisButtonPressed = "right_stick_y";
            }
            else if (gamepad1.dpad_up) {
                thisButtonPressed = "dpad_up";
            }
            else if (gamepad1.dpad_down) {
                thisButtonPressed = "dpad_down";
            }
            else if (gamepad1.dpad_right) {
                thisButtonPressed = "dpad_right";
            }
            else if (gamepad1.dpad_left) {
                thisButtonPressed = "dpad_left";
            }
            else if (gamepad1.start) {
                thisButtonPressed = "start";
            }
            else if (gamepad1.guide) {
                thisButtonPressed = "guide";
            }
            else if (gamepad1.right_trigger != 0.0) {
                thisButtonPressed = "right_trigger";
            }
            else if (gamepad1.left_trigger != 0.0) {
                thisButtonPressed = "left_trigger";
            }
            else {
                thisButtonPressed = "";
            }
            
            moveArmBy = -(int)(gamepad1.left_stick_y * 20);
            moveWristBy = -(int)(gamepad1.right_stick_y * 20);
            
            // make sure an action only runs once per button press
            if (!lastButtonPressed.equals(thisButtonPressed)) {
                if (!thisButtonPressed.equals("")) {
                    towerThread.interrupt();
                }
                switch (thisButtonPressed) {
                    case "a": {
                        lastTowerAction = TowerAction.GO_TO_INTAKE;
                    }
                    break;
                    case "b": {
                        lastTowerAction = TowerAction.INIT;
                    }
                    break;
                    case "x": {
                        lastTowerAction = TowerAction.SCORE_HIGH_BASKET;
                    }
                    break;
                    case "y": {
                        lastTowerAction = TowerAction.SCORE_SPECIMEN_HIGH;
                    }
                    break;
                    case "left_stick_y": {
                        lastTowerAction = TowerAction.MANUAL_MOVE_ARM;
                    }
                    break;
                    case "right_stick_y": {
                        lastTowerAction = TowerAction.MANUAL_MOVE_WRIST;
                    }
                    break;
                    case "dpad_up": {
                        lastTowerAction = TowerAction.MANUAL_RAISE_LEVER;
                    }
                    break;
                    case "dpad_down": {
                        lastTowerAction = TowerAction.MANUAL_LOWER_LEVER;
                    }
                    break;
                    case "dpad_left": {
                        lastTowerAction = TowerAction.BEFORE_SPECIMEN_HIGH;
                    }
                    break;
                    case "dpad_right": {
                        lastTowerAction = TowerAction.GRAB_SPECIMEN_WALL;
                    }
                    break;
                    case "start": {
                        lastTowerAction = TowerAction.ASCEND_LEVEL_TWO;
                    }
                    break;
                    case "right_trigger": {
                        lastTowerAction = TowerAction.MANUAL_INTAKE;
                    }
                    break;
                    case "left_trigger": {
                        lastTowerAction = TowerAction.MANUAL_OUTTAKE;
                    }
                    break;
                    default: {
                        lastTowerAction = TowerAction.IDLE;
                    }
                    break;
                }
            }
            
            telemetry.addData("Last Tower Action", lastTowerAction.name());
            lastTowerAction.run();
            
            lastButtonPressed = thisButtonPressed;
            
            // DRIVE Split Arcade
            y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed!
            x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = gamepad2.right_stick_x;
            
            // if any joystick is being used, set the drivetrain action to drive else idle
            if (x != 0.0 || y != 0.0 || rx != 0.0) {
                if (lastDrivetrainAction != DrivetrainAction.DRIVE) {
                    drivetrainThread.interrupt();
                }
                lastDrivetrainAction = DrivetrainAction.DRIVE;
            }
            else {
                if (lastDrivetrainAction != DrivetrainAction.IDLE) {
                    drivetrainThread.interrupt();
                }
                lastDrivetrainAction = DrivetrainAction.IDLE;
            }
            lastDrivetrainAction.run();
            
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                if (lastClawAction != ClawAction.OPEN) {
                    clawThread.interrupt();
                }
                lastClawAction = ClawAction.OPEN;
            }
            else {
                if (lastClawAction != ClawAction.CLOSE) {
                    clawThread.interrupt();
                }
                lastClawAction = ClawAction.CLOSE;
            }
            lastClawAction.run();
            
            telemetry.addData("Last Drivetrain Action", lastDrivetrainAction.name());
            telemetry.addData("Last Tower Action", lastTowerAction.name());
            telemetry.addData("Last Claw Action", lastClawAction.name());
            telemetry.addData("Claw Open?", clawOpen);
            telemetry.addData("Claw Position Double", claw.getPosition());
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Wrist Power", wrist.getPower());
            telemetry.addData("Lever Position", lever.getCurrentPosition());
            telemetry.addData("Lever Power", lever.getPower());
            telemetry.addData("Target Arm", targetArm);
            telemetry.addData("Target Wrist", targetWrist);
            telemetry.addData("Target Lever", targetLever);
            telemetry.update();

            towerThread = new Thread(lastTowerAction);
            drivetrainThread = new Thread(lastDrivetrainAction);
            clawThread = new Thread(lastClawAction);

            towerThread.start();
            drivetrainThread.start();
            clawThread.start();
            
        }
    }
    
    public static void toggleClaw () {
        if (clawOpen) {
            claw.setPosition(1.0);
        }
        else {
            claw.setPosition(0.5);
        }
        clawOpen = !clawOpen;
    }
    
    public static void setClaw (boolean open) {
        if (open) {
            claw.setPosition(0.35);
        }
        else {
            claw.setPosition(1.0);
        }
        clawOpen = open;
    }
    
    public static void moveArmTo (int pos, double power) {
        targetArm = pos;
        armPower = power;
        
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        
        if (targetArm > 3765) {
            targetArm = 3765;
        }
        else if (targetArm < 0) {
            targetArm = 0;
        }
        
        while (arm.isBusy() &&
        !(pos - armMarginOfError < arm.getCurrentPosition() && arm.getCurrentPosition() < pos + armMarginOfError)) {
            
        }
    }

    public static void moveArmTo (int pos) {
        targetArm = pos;
        armPower = 1.0;
        
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        
        if (targetArm > 3765) {
            targetArm = 3765;
        }
        else if (targetArm < 0) {
            targetArm = 0;
        }
        
        while (arm.isBusy() &&
        !(pos - armMarginOfError < arm.getCurrentPosition() && arm.getCurrentPosition() < pos + armMarginOfError)) {
            
        }
    }
    
    public static void moveWristTo (int pos, double power) {
        targetWrist = pos;
        wristPower = power;
        
        wrist.setTargetPosition(targetWrist);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(wristPower);
        
        if (targetWrist > 2000) {
            targetWrist = 2000;
        }
        else if (targetArm < 0) {
            targetWrist = 0;
        }
        
        while (wrist.isBusy() && !(pos - wristMarginOfError < wrist.getCurrentPosition() && wrist.getCurrentPosition() < pos + wristMarginOfError)) {
            
        }
    }

    public static void moveWristTo (int pos) {
        targetWrist = pos;
        wristPower = 0.75;
        
        wrist.setTargetPosition(targetWrist);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(wristPower);
        
        if (targetWrist > 2000) {
            targetWrist = 2000;
        }
        else if (targetArm < 0) {
            targetWrist = 0;
        }
        
        while (wrist.isBusy() && !(pos - wristMarginOfError < wrist.getCurrentPosition() && wrist.getCurrentPosition() < pos + wristMarginOfError)) {
            
        }
    }
    
    public static void moveLeverTo (int pos, double power) {
        targetLever = pos;
        leverPower = power;
        
        lever.setTargetPosition(targetLever);
        lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lever.setPower(leverPower);
        
        if (targetLever > 600) {
            targetLever = 600;
        }
        else if (targetLever < 0) {
            targetLever = 0;
        }
        
        while (lever.isBusy() && !(pos - leverMarginOfError < lever.getCurrentPosition() && lever.getCurrentPosition() < pos + leverMarginOfError)) {
            
        }
    }
    
    public static void moveLeverTo (int pos) {
        targetLever = pos;
        leverPower = 0.75;
        
        lever.setTargetPosition(targetLever);
        lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lever.setPower(leverPower);
        
        if (targetLever > 600) {
            targetLever = 600;
        }
        else if (targetLever < 0) {
            targetLever = 0;
        }
        
        while (lever.isBusy() && !(pos - leverMarginOfError < lever.getCurrentPosition() && lever.getCurrentPosition() < pos + leverMarginOfError)) {
            
        }
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
