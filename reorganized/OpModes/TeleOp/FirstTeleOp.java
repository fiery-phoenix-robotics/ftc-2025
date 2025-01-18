package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FirstTeleOp", group="Development")
public class FirstTeleOp extends LinearOpMode {
    
    public Drivetrain drivetrain = new Drivetrain();

    @Override
    public void runOpMode () {

        drivetrain.leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        drivetrain.rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        drivetrain.leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        drivetrain.rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.TeleOp.doMotion(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            drivetrain.updateTelemetry();
            telemetry.update();
        }

    }
}