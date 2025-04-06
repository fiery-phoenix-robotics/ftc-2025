package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.*;

import org.firstinspires.ftc.lib.geometry.*;

@TeleOp(name = "FirstTeleOp", group = "Development")
public class FirstTeleOp extends OpMode {

    public Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init () {

        gamepad1.setJoystickDeadzone(Constants.Gamepad1.joystickDeadzone);
        gamepad2.setJoystickDeadzone(Constants.Gamepad2.joystickDeadzone);

        Subsystem.hardwareMap = hardwareMap;
        Subsystem.telemetry = telemetry;

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

        drivetrain.loop();

        Trajectory2d t = new Trajectory2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.drive(t);

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}