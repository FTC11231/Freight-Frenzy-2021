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

	/**
	 * Initializes the carousel.
	 *
	 * @param opMode OpMode for telemetry.
	 */
	public Carousel(OpMode opMode) {
		this.opMode = opMode;
		this.motor = opMode.hardwareMap.get(DcMotorEx.class, "carousel");
		this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.Carousel.pidfCoefficients);
	}

	/**
	 * Initializes the carousel.
	 *
	 * @param linearOpMode LinearOpMode for telemetry.
	 */
	public Carousel(LinearOpMode linearOpMode) {
		this.linearOpMode = linearOpMode;
		this.motor = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "carousel");
		this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.Carousel.pidfCoefficients);
	}

	/**
	 * Sets the velocity of the carousel wheel.
	 *
	 * @param rpm RPM that the wheel should spin at.
	 */
	public void setVelocity(double rpm) {
		this.motor.setVelocity(rpm * Constants.Carousel.rpmToTicks);
	}

	/**
	 * Gets the velocity of the carousel wheel.
	 *
	 * @return The velocity of the wheel.
	 */
	public double getVelocity() {
		return this.motor.getVelocity();
	}

}
