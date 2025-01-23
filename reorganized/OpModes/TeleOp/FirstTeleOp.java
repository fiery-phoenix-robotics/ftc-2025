package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp(name="FirstTeleOp", group="Development")
public class FirstTeleOp extends OpMode {
    
    public Drivetrain drivetrain = new Drivetrain();
    
    @Override
    public void init () {

        drivetrain.leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        drivetrain.rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        drivetrain.leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        drivetrain.rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        drivetrain.otis = hardwareMap.get(SparkFunOTOS.class, "otis");

        drivetrain.init();

    }

    @Override
    public void init_loop () {

    }

    @Override
    public void start () {
        
    }

    @Override
    public void loop () {

        drivetrain.TeleOp.doMotion(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
        drivetrain.updateTelemetry(telemetry);

        telemetry.update();

    }

    @Override
    public void stop () {
        
    }

}