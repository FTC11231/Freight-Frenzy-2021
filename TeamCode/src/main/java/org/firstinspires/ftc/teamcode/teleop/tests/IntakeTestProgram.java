package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Test", group = "Iterative Opmode")
public class IntakeTestProgram extends OpMode {

    private String versionNumber = "v0.1'";

    private DcMotor intake;
    private Servo gripper;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "lm1");
        gripper = hardwareMap.get(Servo.class, "gripper");

        telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            intake.setPower(1);
        else if (gamepad1.b)
            intake.setPower(-1);
        else
            intake.setPower(0);
        if (gamepad1.x)
            gripper.setPosition(0);
        else if (gamepad1.y)
            gripper.setPosition(1);
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
        telemetry.update();
    }

}
