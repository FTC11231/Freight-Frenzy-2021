package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.AutonomousParameter;
import org.firstinspires.ftc.teamcode.Utils.Hardware;

@Autonomous(name = "Selection Menu", group = "Linear Opmode")
@Disabled
public class AutonomousSelectionMenu extends LinearOpMode {

	private String versionNumber = "v0.1";
	private Hardware robot = new Hardware(this, telemetry);

	private AutonomousParameter[] parameters = {
		new AutonomousParameter("Delay", 0, this.telemetry),
		new AutonomousParameter("Random Int", 0, this.telemetry),
		new AutonomousParameter("Random Bool", false, this.telemetry)
	};

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		boolean upPressedLast = gamepad1.dpad_up;
		boolean downPressedLast = gamepad1.dpad_down;
		boolean leftPressedLast = gamepad1.dpad_left;
		boolean rightPressedLast = gamepad1.dpad_right;
		boolean aPressedLast = gamepad1.a;
		boolean bPressedLast = gamepad1.b;
		int selected = 0;

		while (!opModeIsActive() || !isStopRequested()) {
			for (int i = 0; i < parameters.length; i++) {
				if (i == selected)
					parameters[i].setSelected(true);
				else
					parameters[i].setSelected(false);
			}
			boolean uDown = gamepad1.dpad_up && !upPressedLast;
			boolean dDown = gamepad1.dpad_down && !downPressedLast;
			boolean lDown = gamepad1.dpad_left && !leftPressedLast;
			boolean rDown = gamepad1.dpad_right && !rightPressedLast;
			boolean aDown = gamepad1.a && !aPressedLast;
			boolean bDown = gamepad1.b && !bPressedLast;
			if (uDown)
				selected--;
			if (dDown)
				selected++;
			selected = Math.max(0, selected);
			if (rDown)
				this.parameters[selected].incrementVal(1);
			if (lDown)
				this.parameters[selected].incrementVal(-1);
			if (aDown) {
				this.parameters[selected].incrementVal(1);
				this.parameters[selected].setVal(true);
			}
			if (bDown) {
				this.parameters[selected].incrementVal(-1);
				this.parameters[selected].setVal(false);
			}
			for (int i = 0; i < parameters.length; i++) {
				parameters[i].display();
			}
		}

		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");

		robot.resetAngle();

		// Autonomous goes here

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}
