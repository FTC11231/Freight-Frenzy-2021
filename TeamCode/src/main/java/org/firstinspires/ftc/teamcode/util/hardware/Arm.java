package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {

    public DcMotor motorOne;
    public DcMotor motorTwo;
    public OpMode opMode;
    public LinearOpMode linearOpMode;

    public Arm(OpMode opMode) {
        this.opMode = opMode;
        this.motorOne = this.opMode.hardwareMap.get(DcMotor.class, "armOne");
        this.motorTwo = this.opMode.hardwareMap.get(DcMotor.class, "armTwo");
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Arm(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.motorOne = this.linearOpMode.hardwareMap.get(DcMotor.class, "armOne");
        this.motorTwo = this.linearOpMode.hardwareMap.get(DcMotor.class, "armTwo");
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        this.motorOne.setPower(power);
        this.motorOne.setPower(-power);
    }

}
