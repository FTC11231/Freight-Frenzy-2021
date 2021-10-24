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
	 * Initializes the turret.
	 *
	 * @param linearOpMode LinearOpMode for telemetry.
	 * @param reset        If true, the encoder position will be reset upon initialization
	 */
	public Turret(LinearOpMode linearOpMode, boolean reset) {
		this.linearOpMode = linearOpMode;
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
	public void turn(double degrees) {
		motor.setTargetPosition((int) (Constants.Turret.TICKS_PER_DEGREE * degrees));
		motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, Constants.Turret.PIDF_COEFFICIENTS);
		motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		motor.setPower(0.3);
	}

	/**
	 * Gets the angle that the turret is facing.
	 *
	 * @return Angle, in degrees, that the turret is facing.
	 */
	public double getRotation() {
		return motor.getCurrentPosition() / Constants.Turret.TICKS_PER_DEGREE;
	}

	/**
	 * Resets the encoder position of the turret.
	 */
	public void resetEncoder() {
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

}
