package org.firstinspires.ftc.teamcode;

import java.util.Map;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@Autonomous(name = "FirstAuto", group = "Development")
public class FirstAuto extends OpMode {

    public Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init () {

        Subsystem.hardwareMap = hardwareMap;

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

        drivetrain.updateTelemetry(telemetry);

        telemetry.update();

    }

    @Override
    public void stop () {

    }

}