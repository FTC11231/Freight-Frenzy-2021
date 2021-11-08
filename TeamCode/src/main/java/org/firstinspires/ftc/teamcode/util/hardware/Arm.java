package org.firstinspires.ftc.teamcode.util.hardware;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Arm {

	public DcMotorEx motorOne;
	public DcMotorEx motorTwo;
	public OpMode opMode;
	public double desiredAngle;
	public double commandedAngle;

	public enum Position {
		INTAKE(2),
		LEVEL_ONE(17),
		LEVEL_TWO(49),
		LEVEL_THREE(70);

		double degrees;

		Position(double degrees) {
			this.degrees = degrees;
		}

		public double getDegrees() {
			return degrees;
		}

	}

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
		this.motorOne.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, Constants.Arm.MOTOR_ONE_PIDF);
		this.motorTwo.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, Constants.Arm.MOTOR_TWO_PIDF);
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	public void update() {
		commandedAngle = commandedAngle + MathUtils.clamp(desiredAngle - commandedAngle,
				-Constants.Arm.CHANGE_RATE_LOWER_LIMIT, Constants.Arm.CHANGE_RATE_UPPER_LIMIT);
		opMode.telemetry.addData("Commanded arm angle", commandedAngle);
		this.motorOne.setTargetPosition((int) -(commandedAngle * Constants.Arm.TICKS_PER_DEGREE));
		this.motorTwo.setTargetPosition((int) (commandedAngle * Constants.Arm.TICKS_PER_DEGREE));
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		this.motorOne.setPower(1);
		this.motorTwo.setPower(1);
	}

	/**
	 * Turns the arm to the given angle.
	 *
	 * @param degrees The angle the arm will turn to.
	 */
	public void setPosition(double degrees) {
//		this.motorOne.setTargetPosition((int) -(degrees * Constants.Arm.TICKS_PER_DEGREE));
//		this.motorTwo.setTargetPosition((int) (degrees * Constants.Arm.TICKS_PER_DEGREE));
//		this.motorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//		this.motorOne.setPower(1);
//		this.motorTwo.setPower(1);
		desiredAngle = degrees;
	}

	/**
	 * Turns the arm to the given position.
	 *
	 * @param position The position (enum) that the arm will turn to.
	 */
	public void setPosition(Position position) {
//		this.motorOne.setTargetPosition((int) -(position.getDegrees() * Constants.Arm.TICKS_PER_DEGREE));
//		this.motorTwo.setTargetPosition((int) (position.getDegrees() * Constants.Arm.TICKS_PER_DEGREE));
//		this.motorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//		this.motorOne.setPower(1);
//		this.motorTwo.setPower(1);
		desiredAngle = position.getDegrees();
	}

	/**
	 * Gets the angle that the arm is facing.
	 *
	 * @return Angle, in degrees, that the arm is facing.
	 */
	public double getRotation() {
		return ((motorOne.getCurrentPosition() / Constants.Arm.TICKS_PER_DEGREE)
				- (motorTwo.getCurrentPosition() / Constants.Arm.TICKS_PER_DEGREE)) / -2;
	}

	/**
	 * Resets the encoder position of the turret.
	 */
	public void resetEncoders() {
		this.motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	/**
	 * Sets the power of the arm.
	 *
	 * @param power The power the arm will run at.
	 */
	public void setPower(double power) {
		this.motorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		this.motorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		this.motorOne.setPower(power);
		this.motorTwo.setPower(-power);
	}

}
