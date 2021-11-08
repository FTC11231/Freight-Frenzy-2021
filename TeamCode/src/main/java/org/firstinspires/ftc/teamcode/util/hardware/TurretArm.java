package org.firstinspires.ftc.teamcode.util.hardware;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TurretArm {

	private OpMode opMode;
	public Arm arm;
	public Turret turret;

	private double turretTargetPosition;
	private double armTargetPosition;

	private double[] turretSpaceBorders = {50, 96.25, 82.5, 128.75};
	/*
	 *
	 *       128.75 | \ | 82.5     96.25 | / | 50
	 *          (3) | \ | (2)        (1) | / | (0)
	 *              | \ |                | / |
	 *              |   |                |   |
	 *              |   |                |   |
	 *              |   | -------------- |   |
	 *              |   | [   /----\   ] |   |
	 *              | / | [  |      |  ] | \ |
	 *              | / | [   \____/   ] | \ |
	 *              | / | -------------- | \ |
	 *
	 */

	public TurretArm(OpMode opMode) {
		this.opMode = opMode;
		turret = new Turret(this.opMode, true);
		arm = new Arm(this.opMode, true);
	}

	public void update() {
		int armBadHeight = 30;

		// If turret or target position is not in a pocket, or target pos is in a different space, raise arm
		// If turret target pos is in a different space AND turret is in pocket AND arm is down
		//      constrain turret to current pocket
		//
		double armTargetPositionReal = armTargetPosition;
		double turretTargetPositionReal = turretTargetPosition;

		if (getTurretSpaceNumber(turret.getRotation()) % 2 != 0 // If turret is in a no-go-zone
				|| getTurretSpaceNumber(turret.getRotation()) != getTurretSpaceNumber(turretTargetPosition)) { // Or turret is trying to move spaces
			armTargetPositionReal = Math.max(armTargetPosition, armBadHeight + 5); // Raise arm to go over no-go-zone
		}
		if (getTurretSpaceNumber(turret.getRotation()) != getTurretSpaceNumber(turretTargetPosition) // If turret is trying to move spaces
				&& getTurretSpaceNumber(turret.getRotation()) % 2 == 0 // And turret is in a pocket
				&& arm.getRotation() <= armBadHeight) { // And arm is down
			// Constrain the turret
			int spaceNumber = getTurretSpaceNumber(turret.getRotation());
			if (spaceNumber == 0) {
				turretTargetPositionReal = Math.min(turretTargetPosition, turretSpaceBorders[spaceNumber]);
			} else if (spaceNumber == turretSpaceBorders.length) {
				turretTargetPositionReal = Math.max(turretTargetPosition, turretSpaceBorders[spaceNumber - 1]);
			} else {
				turretTargetPositionReal = MathUtils.clamp(turretTargetPosition, turretSpaceBorders[spaceNumber - 1], turretSpaceBorders[spaceNumber]);
			}
		}
		opMode.telemetry.addData("Turret space", getTurretSpaceNumber(turret.getRotation()));
//		arm.setPosition(armTargetPositionReal);
//		turret.setPosition(turretTargetPositionReal);
	}

	public void setTurretPosition(double degrees) {
		turretTargetPosition = degrees;
	}

	public void setTurretPosition(Turret.Position position) {
		turretTargetPosition = position.getDegrees();
	}

	public void setArmPosition(double degrees) {
		armTargetPosition = degrees;
	}

	public double getArmAngle() {
		return arm.getRotation();
	}

	public double getArmTargetPosition() {
		return armTargetPosition;
	}

	public double getTurretAngle() {
		return turret.getRotation();
	}

	public double getTurretTargetPosition() {
		return turretTargetPosition;
	}

	public void setArmPosition(Arm.Position position) {
		armTargetPosition = position.getDegrees();
	}

	public int getTurretSpaceNumber(double degrees) {
		// Will return the index of the space the turret is in, in order of:
		// Left pocket, front left no-go-zone, front pocket, front right no-go-zone, right pocket
		// 0          , 1                    , 2           , 3                     , 4
		// Even number = pocket, odd number = no-go-zone
		if (degrees <= turretSpaceBorders[0]) {
			return 0; // Right pocket
		} else if (degrees <= turretSpaceBorders[1]) {
			return 1; // Front-right no-go-zone
		} else if (degrees <= turretSpaceBorders[2]) {
			return 2; // Front pocket
		} else if (degrees <= turretSpaceBorders[3]) {
			return 3; // Front-left no-go-zone
		} else if (degrees <= turretSpaceBorders[4]){
			return 4; // Left pocket
		}
		return -1; // Something went wrong
	}

}
