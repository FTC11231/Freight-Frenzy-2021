package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Hardware;

@TeleOp(name = "Field-Centric Tele-Op", group = "Iterative Opmode")
public class FieldCentricTeleOp extends OpMode {

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
        double joystickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x); // Get the joystick angle
        joystickAngle -= robot.getAngleRadians(); // Subtract the angle of the robot to get the
        joystickAngle += 0; // Gyro angle is reset at the start of autonomous, meaning that 0 = away from humans
        double newDrive = Math.sin(joystickAngle); // Set the new drive variable
        double newStrafe = Math.cos(joystickAngle); // Set the new strafe variable
        robot.drive(newDrive, newStrafe, gamepad1.right_stick_x); // Drive with field centric stuff
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
        telemetry.update();
    }

}
