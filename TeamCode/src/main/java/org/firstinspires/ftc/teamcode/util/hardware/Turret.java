package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Turret {

    public DcMotor motor;
    public LinearOpMode linearOpMode;
    public OpMode opMode;

    public Turret(OpMode opMode) {
        this.opMode = opMode;
        this.motor = opMode.hardwareMap.get(DcMotor.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Turret(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.motor = opMode.hardwareMap.get(DcMotor.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToPosition(int degrees) {
        motor.setTargetPosition((int) Constants.Turret.ticksPerDegree * degrees);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
