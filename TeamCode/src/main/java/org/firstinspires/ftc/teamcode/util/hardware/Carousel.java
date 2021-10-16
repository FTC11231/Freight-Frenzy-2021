package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Carousel {

    public DcMotorEx motor;
    public OpMode opMode;
    public LinearOpMode linearOpMode;

    public Carousel(OpMode opMode) {
        this.opMode = opMode;
        this.motor = opMode.hardwareMap.get(DcMotorEx.class, "carousel");
        this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.Carousel.pidfCoefficients);
    }

    public Carousel(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.motor = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "carousel");
        this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.Carousel.pidfCoefficients);
    }

    public void setVelocity(double rpm) {
        this.motor.setVelocity(rpm * Constants.Carousel.rpmToTicks);
    }

    public double getVelocity() {
        return this.motor.getVelocity();
    }

}
