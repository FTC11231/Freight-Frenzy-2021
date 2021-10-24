package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Arm {

	public DcMotorEx motorOne;
	public DcMotorEx motorTwo;
	public OpMode opMode;
	public LinearOpMode linearOpMode;

	/**
	 * Initializes the arm.
	 *
	 * @param opMode OpMode for telemetry.
	 * @param reset  If true, the encoder position will be reset upon initialization
	 */
	public Arm(OpMode opMode, boolean reset) {
		this.opMode = opMode;
		this.motorOne = this.opMode.hardwareMap.get(DcMotorEx.class, "armOne");
		this.motorTwo = this.opMode.hardwareMap.get(DcMotorEx.class, "armTwo");
		if (reset) {
			this.motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * Initializes the arm.
	 *
	 * @param linearOpMode OpMode for telemetry.
	 * @param reset        If true, the encoder position will be reset upon initialization
	 */
	public Arm(LinearOpMode linearOpMode, boolean reset) {
		this.linearOpMode = linearOpMode;
		this.motorOne = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "armOne");
		this.motorTwo = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "armTwo");
		if (reset) {
			this.motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * Turns the arm to the given angle.
	 *
	 * @param degrees The angle the turret will turn to.
	 */
	public void turn(double degrees) {
		this.motorOne.setTargetPosition((int) (Constants.Arm.TICKS_PER_DEGREE * degrees));
		this.motorTwo.setTargetPosition((int) (Constants.Arm.TICKS_PER_DEGREE * degrees));
		this.motorOne.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, Constants.Arm.PIDF_COEFFICIENTS);
		this.motorTwo.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, Constants.Arm.PIDF_COEFFICIENTS);
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		this.motorOne.setPower(0.7);
		this.motorTwo.setPower(0.7);
	}

	/**
	 * Gets the angle that the arm is facing.
	 *
	 * @return Angle, in degrees, that the arm is facing.
	 */
	public double getRotation() {
		return ((motorOne.getCurrentPosition() / Constants.Turret.TICKS_PER_DEGREE) + (motorOne.getCurrentPosition() / Constants.Turret.TICKS_PER_DEGREE)) / 2;
	}

	/**
	 * Resets the encoder position of the turret.
	 */
	public void resetEncoders() {
		this.motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	/**
	 * Sets the power of the arm.
	 *
	 * @param power The power the arm will run at.
	 */
	public void setPower(double power) {
		this.motorOne.setPower(power);
		this.motorOne.setPower(power);
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

}
