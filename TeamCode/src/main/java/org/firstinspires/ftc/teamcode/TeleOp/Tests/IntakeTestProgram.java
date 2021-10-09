package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Hardware;

@TeleOp(name = "Intake Test", group = "Iterative Opmode")
public class IntakeTestProgram extends OpMode {

    private String versionNumber = "v0.1'";
    private Hardware robot = new Hardware(this, this.telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);

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
            robot.lm1.setPower(1);
        else if (gamepad1.b)
            robot.lm1.setPower(-1);
        else
            robot.lm1.setPower(0);
        if (gamepad1.x)
            robot.servo.setPosition(0);
        else if (gamepad1.y)
            robot.servo.setPosition(1);
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
        telemetry.update();
    }

}
