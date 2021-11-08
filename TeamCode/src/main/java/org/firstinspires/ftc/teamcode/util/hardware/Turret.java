package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Turret {

	public DcMotorEx motor;
	public LinearOpMode linearOpMode;
	public OpMode opMode;

	public enum Position {
		RIGHT(0),
		FRONT(90),
		LEFT(180),
		BACK(-45);

		double degrees;

		Position(double degrees) {
			this.degrees = degrees;
		}

		public double getDegrees() {
			return degrees;
		}

	}

	/**
	 * Initializes the turret.
	 *
	 * @param opMode OpMode for telemetry.
	 * @param reset  If true, the encoder position will be reset upon initialization
	 */
	public Turret(OpMode opMode, boolean reset) {
		this.opMode = opMode;
		this.motor = opMode.hardwareMap.get(DcMotorEx.class, "turret");
		if (reset) {
			motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		}
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, Constants.Turret.PIDF_COEFFICIENTS);
	}

	/**
	 * Turns the turret to the given angle.
	 *
	 * @param degrees The angle the turret will turn to.
	 */
	public void setPosition(double degrees) {
		motor.setTargetPosition((int) -((degrees - 90) * Constants.Turret.TICKS_PER_DEGREE));
//		motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, Constants.Turret.PIDF_COEFFICIENTS);
		motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	/**
	 * Turns the turret to the given position.
	 *
	 * @param position The position (enum) the turret will turn to.
	 */
	public void setPosition(Position position) {
		motor.setTargetPosition((int) -((position.getDegrees() - 90) * Constants.Turret.TICKS_PER_DEGREE));
		motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	/**
	 * Gets the angle that the turret is facing.
	 *
	 * @return Angle, in degrees, that the turret is facing.
	 */
	public double getRotation() {
		return -motor.getCurrentPosition() / Constants.Turret.TICKS_PER_DEGREE + 90;
	}

	/**
	 * Resets the encoder position of the turret.
	 */
	public void resetEncoder() {
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

}
