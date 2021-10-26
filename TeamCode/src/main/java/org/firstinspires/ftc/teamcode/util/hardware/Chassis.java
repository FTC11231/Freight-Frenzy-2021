package org.firstinspires.ftc.teamcode.util.hardware;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Chassis {

	public DcMotor lm1, lm2, rm1, rm2;
	public BNO055IMU imu;
	public LinearOpMode linearOpMode;
	public OpMode opMode;

	// Sensor related variables
	Orientation lastAngles = new Orientation();
	double globalAngle;

	/**
	 * Initializes the chassis.
	 *
	 * @param linearOpMode OpMode for telemetry.
	 */
	public Chassis(LinearOpMode linearOpMode) {
		this.linearOpMode = linearOpMode;

		this.lm1 = this.linearOpMode.hardwareMap.get(DcMotor.class, "lm1");
		this.lm2 = this.linearOpMode.hardwareMap.get(DcMotor.class, "lm2");
		this.rm1 = this.linearOpMode.hardwareMap.get(DcMotor.class, "rm1");
		this.rm2 = this.linearOpMode.hardwareMap.get(DcMotor.class, "rm2");

		this.lm1.setDirection(DcMotorSimple.Direction.FORWARD);
		this.lm2.setDirection(DcMotorSimple.Direction.FORWARD);
		this.rm1.setDirection(DcMotorSimple.Direction.REVERSE);
		this.rm2.setDirection(DcMotorSimple.Direction.REVERSE);

//		// Initialize IMU
//		this.imu = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		parameters.loggingEnabled = false;
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//		this.imu.initialize(parameters);

		drive(0, 0, 0);
	}

	/**
	 * Initializes the chassis.
	 *
	 * @param opMode OpMode for telemetry.
	 */
	public Chassis(OpMode opMode) {
		this.opMode = opMode;

		this.lm1 = this.opMode.hardwareMap.get(DcMotor.class, "lm1");
		this.lm2 = this.opMode.hardwareMap.get(DcMotor.class, "lm2");
		this.rm1 = this.opMode.hardwareMap.get(DcMotor.class, "rm1");
		this.rm2 = this.opMode.hardwareMap.get(DcMotor.class, "rm2");

		this.lm1.setDirection(DcMotorSimple.Direction.FORWARD);
		this.lm2.setDirection(DcMotorSimple.Direction.FORWARD);
		this.rm1.setDirection(DcMotorSimple.Direction.REVERSE);
		this.rm2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//		// Initialize IMU
//		this.imu = this.linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");
//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		parameters.loggingEnabled = false;
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//		this.imu.initialize(parameters);

		drive(0, 0, 0);
	}

	/**
	 * Drives with the parameters given.
	 *
	 * @param drive  Forwards.
	 * @param strafe Right.
	 * @param turn   Clockwise.
	 */
	public void drive(double drive, double strafe, double turn) {
		lm1.setPower(drive + strafe + turn);
		lm2.setPower(drive - strafe + turn);
		rm1.setPower(drive - strafe - turn);
		rm2.setPower(drive + strafe - turn);
	}

	/**
	 * Gets the angle in degrees of the robot.
	 *
	 * @return Returns the angle in degrees of the robot.
	 */
	public double getAngle() {
		// We experimentally determined the Z axis is the axis we want to use for heading angle.
		// We have to process the angle because the imu works in euler angles so the Z axis is
		// returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
		// 180 degrees. We detect this transition and track the total cumulative angle of rotation.

		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

		if (deltaAngle < -180)
			deltaAngle += 360;
		else if (deltaAngle > 180)
			deltaAngle -= 360;

		globalAngle += deltaAngle;

		lastAngles = angles;

		return globalAngle;
	}

	/**
	 * Gets the angle in radians of the robot.
	 *
	 * @return Returns the angle in radians of the robot.
	 */
	public double getAngleRadians() {
		return Math.toRadians(getAngle());
	}

	/**
	 * Zero's the IMU of the robot.
	 */
	public void resetAngle() {
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		globalAngle = 0;
	}

	/**
	 * Sets the RunMode of the drive motors.
	 *
	 * @param runMode The RunMode the motors are being set to.
	 */
	private void setRunMode(DcMotor.RunMode runMode) {
		lm1.setMode(runMode);
		lm2.setMode(runMode);
		rm1.setMode(runMode);
		rm2.setMode(runMode);
	}

	/**
	 * Gets the average position of the drive encoders.
	 *
	 * @return Returns the average position of the drive encoders.
	 */
	private double getDrivePosition() {
		return (Math.abs(lm1.getCurrentPosition())
				+ Math.abs(lm2.getCurrentPosition())
				+ Math.abs(rm1.getCurrentPosition())
				+ Math.abs(rm2.getCurrentPosition())) / 4;
	}

	/**
	 * Drives the robot forward.
	 *
	 * @param distance          The distance the robot will travel in inches.
	 * @param maxSpeed          The maxSpeed the robot will travel in motor power (0-1).
	 * @param speedUpPercentage The percentage the distance will be at before starting to ramp up or down.
	 */
	public void driveForward(double distance, double maxSpeed, double speedUpPercentage, double slowDownPercentage) {
		linearOpMode.telemetry.addData("Status", "Driving for " + distance + " inches");
		linearOpMode.telemetry.update();

		speedUpPercentage = Math.abs(speedUpPercentage);
		slowDownPercentage = Math.abs(slowDownPercentage);
		maxSpeed = Math.abs(maxSpeed);
		if (distance == 0) return;
		boolean forwards = distance > 0;
		distance *= Constants.Drivetrain.TICKS_PER_INCH; // Sets it to be the distance in ticks
		distance = Math.abs(distance);

		setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

		double percentage;
		double power;
		// Stop if A is pressed for debugging incase it's really not good
		while (Math.abs(getDrivePosition()) < distance && this.linearOpMode.opModeIsActive() && !this.linearOpMode.gamepad1.a) {
			percentage = Math.abs(getDrivePosition() / distance);
			if (percentage <= speedUpPercentage) {
				// Speed up
				power = (maxSpeed / speedUpPercentage) * percentage;
			} else if (percentage >= 1 - slowDownPercentage) {
				// Slow down
				power = -(maxSpeed / slowDownPercentage) * percentage;
			} else {
				// Full power
				power = maxSpeed;
			}
			power = MathUtils.clamp(power, 0.1, maxSpeed);
			if (forwards) {
				// Drive forwards
				drive(power, 0, 0);
			} else {
				// Drive backwards
				drive(-power, 0, 0);
			}
		}
		drive(0, 0, 0);
	}

	/**
	 * Turns the robot to the specified angle.
	 *
	 * @param degrees        The absolute angle the robot will turn to (Forwards from the starting position is 0°, and goes counterclockwise).
	 * @param maxPower       The speed the robot will go at.
	 * @param timeoutSeconds The time the robot will turn for before stopping since the angle is close enough.
	 */
	public void turn(double degrees, double maxPower, double timeoutSeconds) {
		linearOpMode.telemetry.addData("Status", "Turning to " + degrees + "°");
		linearOpMode.telemetry.update();

		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

		PIDController pidController = new PIDController(Constants.Drivetrain.TURNING_PID_KP, Constants.Drivetrain.TURNING_PID_KI, Constants.Drivetrain.TURNING_PID_KD);
		pidController.setTolerance(1);

		double targetAngle = degrees;

		Timer timer = new Timer();
		timer.start();

		loop:
		while (linearOpMode.opModeIsActive()) {
			double pidOutput = pidController.calculate(getAngle(), targetAngle);
			double output = pidOutput + (Math.signum(pidOutput) * Constants.Drivetrain.TURNING_PID_KF * (pidController.atSetpoint() ? 0 : 1));
			drive(0, 0, MathUtils.clamp(-output, -maxPower, maxPower));

			if (pidController.atSetpoint() || timer.hasElapsed(timeoutSeconds)) {
				drive(0, 0, 0);
				break loop;
			}
		}
	}

}
