package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Carousel {

	public DcMotor motor;
	public OpMode opMode;

	/**
	 * Initializes the carousel.
	 *
	 * @param opMode OpMode for telemetry.
	 */
	public Carousel(OpMode opMode) {
		this.opMode = opMode;
		this.motor = this.opMode.hardwareMap.get(DcMotor.class, "carousel");
	}

	/**
	 * Gets the velocity of the carousel wheel.
	 *
	 * @return The velocity of the wheel.
	 */
//	public double getVelocity() {
//		return this.motor.getVelocity() / Constants.Carousel.RPM_TO_TICKS;
//	}

}
