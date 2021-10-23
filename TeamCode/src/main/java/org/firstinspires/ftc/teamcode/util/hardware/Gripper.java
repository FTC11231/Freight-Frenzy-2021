package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

	public Servo servo;
	public LinearOpMode linearOpMode;
	public OpMode opMode;

	/**
	 * Initializes the gripper.
	 *
	 * @param opMode OpMode for telemetry.
	 */
	public Gripper(OpMode opMode) {
		this.opMode = opMode;
		this.servo = opMode.hardwareMap.get(Servo.class, "carousel");
	}

	/**
	 * Initializes the gripper.
	 *
	 * @param linearOpMode LinearOpMode for telemetry.
	 */
	public Gripper(LinearOpMode linearOpMode) {
		this.linearOpMode = linearOpMode;
		this.servo = this.linearOpMode.hardwareMap.get(Servo.class, "carousel");
	}

	/**
	 * Sets the position of the gripper.
	 *
	 * @param position The new position of the gripper.
	 */
	public void setPosition(double position) {
		servo.setPosition(position);
	}

}
