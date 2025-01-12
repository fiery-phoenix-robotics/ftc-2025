// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// 
// @Autonomous
//     
// public class Auto1925 extends LinearOpMode{
//     public static int targetArm = 0;
//     public static int targetWrist = 0;
//     public static int targetLever = 0;
//     
//     private static boolean clawOpen = true;
//     private static boolean clawAlreadyToggled = false;
//     
//     public static double x;
//     public static double y;
//     public static double rx;
//     
//     public static int moveArmBy;
//     public static int moveWristBy;
//     
//     public static int towerActionTimeout = 3000;
//     
//     public static final int ARM_SAMPLE_HIGH = 3690;
//     public static final int ARM_INTAKE = 0;
//     public static final int ARM_INTAKE_SUBMERSIBLE = 395;
//     public static final int ARM_INIT = 0;
//     
//     public static final int WRIST_SAMPLE_HIGH = 1800;
//     public static final int WRIST_INTAKE = 980;
//     public static final int WRIST_INTAKE_SUBMERSIBLE = 1330;
//     public static final int WRIST_INIT = 0;
//     
//     public static final int LEVER_SPECIMEN_WALL = 45;
//     public static final int LEVER_SPECIMEN_HIGH = 290;
//     public static final int LEVER_SPECIMEN_PRESCORE = 465;
//     
//     public static final double POWER = 1.0;
//     
//     @Override
//     public void runOpMode() throws InterruptedException {
//         
//     }
// }
//     
//     public enum TowerAction implements Runnable {
//         // INIT, INTAKE_FLOOR, SCORE_HIGH_BASKET, SCORE_SPECIMEN_HIGH, GRAB_SPECIMEN_WALL,
//         // MANUAL_RAISE_ARM, MANUAL_LOWER_ARM, MANUAL_EXTEND_WRIST, MANUAL_RETRACT_WRIST
//         // MANUAL_INTAKE, MANUAL_OUTTAKE, TOGGLE_CLAW
//         INIT {
//             public void run() {
//                     moveWristTo(0, POWER);
//                     moveArmTo(0, POWER);
//                     moveLeverTo(0, POWER);
//             }
//         },
//         GO_TO_INTAKE {
//             public void run () {
//                     moveArmTo(ARM_INTAKE, POWER);
//                     moveWristTo(WRIST_INTAKE, POWER);
//             }
//         },
//         INTAKE_SUBMERSIBLE {
//             public void run () {
//                 moveArmTo(ARM_INTAKE_SUBMERSIBLE, POWER);
//                 moveWristTo(WRIST_INTAKE_SUBMERSIBLE, POWER);
//             }
//         },
//         SCORE_HIGH_BASKET {
//             public void run () {
//                     moveArmTo(ARM_SAMPLE_HIGH, POWER);
//                     moveWristTo(WRIST_SAMPLE_HIGH, POWER);
//             }
//         },
//         SCORE_SPECIMEN_HIGH {
//             public void run() {
//                     moveWristTo(WRIST_INIT, POWER);
//                     moveArmTo(ARM_INIT, POWER);
//                     moveLeverTo(LEVER_SPECIMEN_HIGH, POWER);
//             }
//         },
//         GRAB_SPECIMEN_WALL {
//             public void run() {
//                 setClaw(true);
//                 moveWristTo(WRIST_INIT, POWER);
//                 moveArmTo(ARM_INIT, POWER);
//                 moveLeverTo(LEVER_SPECIMEN_WALL, POWER);
//             }
//         },
//         BEFORE_SPECIMEN_HIGH {
//             public void run() {
//                 moveWristTo(WRIST_INIT, POWER);
//                 moveArmTo(ARM_INIT, POWER);
//                 moveLeverTo(LEVER_SPECIMEN_PRESCORE, POWER);
//             }
//         },
//         ASCEND_LEVEL_TWO {
//             public void run() {
//                     moveWristTo(0, POWER);
//                     moveArmTo(2500, POWER);
//                     try {
//                         Thread.sleep(2000);
//                     }
//                     catch (InterruptedException e) {
//                         
//                     }
//                     moveArmTo(0, POWER);
//             }
//         },
//         MANUAL_MOVE_ARM {
//             public void run() {
//                 if (0 < arm.getCurrentPosition() + moveArmBy && arm.getCurrentPosition() + moveArmBy < 36765) {
//                     targetArm += moveArmBy;
//                 } else if (arm.getCurrentPosition() < 0){
//                     targetArm = 0;
//                 } else if (arm.getCurrentPosition() > 3765) {
//                     targetArm = 3765;
//                 } else {
//                     return;
//                 }
//                 armPower = 1.0;
//                 
//                 arm.setTargetPosition(targetArm);
//                 arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                 arm.setPower(1.0);
//             }
//         },
//         MANUAL_RAISE_LEVER {
//             public void run() {
//                 targetLever += 2;
//                 leverPower = 1.0;
//                 
//                 lever.setTargetPosition(targetLever);
//                 lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                 lever.setPower(leverPower);
//             }
//         },
//         MANUAL_LOWER_LEVER {
//             public void run() {
//                 targetLever -= 2;
//                 leverPower = 0.5;
//                 
//                 lever.setTargetPosition(targetLever);
//                 lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                 lever.setPower(leverPower);
//             }
//         },
//         MANUAL_MOVE_WRIST {
//             public void run() {
//                 if (0 < wrist.getCurrentPosition() + moveWristBy && wrist.getCurrentPosition() + moveWristBy < 2000) {
//                     targetWrist += moveWristBy;
//                 } else if (wrist.getCurrentPosition() < 0){
//                     targetWrist = 0;
//                 } else if (wrist.getCurrentPosition() > 2000) {
//                     targetWrist = 2000;
//                 } else {
//                     return;
//                 }
//                 wristPower = 1.0;
//                 
//                 wrist.setTargetPosition(targetWrist);
//                 wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                 wrist.setPower(1.0);
//             }
//         },
//         MANUAL_INTAKE {
//             public void run() {
//                     intake.setPower(-1.0);
//             }
//         },
//         MANUAL_OUTTAKE {
//             public void run() {
//                     intake.setPower(1.0);
//             }
//         },
//         IDLE {
//             public void run () {
//                     intake.setPower(0);
//             }
//         };
//     }
//     
//     public enum DrivetrainAction implements Runnable {
//         DRIVE {
//             public void run () {
//                 
//                 // Denominator is the largest motor power (absolute value) or 1
//                 // This ensures all the powers maintain the same ratio,
//                 // but only if at least one is out of the range [-1, 1]
//                 double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                 double frontLeftPower = (y + x + rx) / denominator;
//                 double backLeftPower = (y - x + rx) / denominator;
//                 double frontRightPower = (y - x - rx) / denominator;
//                 double backRightPower = (y + x - rx) / denominator;
//                 
//                 leftDriveFront.setPower(frontLeftPower);
//                 leftDriveRear.setPower(backLeftPower);
//                 rightDriveFront.setPower(frontRightPower);
//                 rightDriveRear.setPower(backRightPower);
//             }
//         },
//         IDLE {
//             public void run () {
//                 leftDriveFront.setPower(0);
//                 leftDriveRear.setPower(0);
//                 rightDriveFront.setPower(0);
//                 rightDriveRear.setPower(0);
//             }
//         }
//     }
//     
//     public enum ClawAction implements Runnable {
//         TOGGLE {
//             public void run() {
//                 toggleClaw();
//             }
//         },
//         OPEN {
//             public void run() {
//                 setClaw(true);
//             }
//         },
//         CLOSE {
//             public void run() {
//                 setClaw(false);
//             }
//         },
//         IDLE {
//             public void run() {
//                 try {
//                     Thread.sleep(1);
//                 }
//                 catch (InterruptedException e) {
//                     
//                 }
//             }
//         }
//     }