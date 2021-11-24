package org.firstinspires.ftc.teamcode.util.hardware;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
	public OpMode opMode;

	// Sensor related variables
	Orientation lastAngles = new Orientation();
	double globalAngle;

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

		// Initialize IMU
		this.imu = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");


		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		parameters.loggingEnabled = false;
//		parameters.loggingEnabled = false;
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		this.imu.initialize(parameters);

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

	public void setTargetPosition(int position) {
		lm1.setTargetPosition(position);
		lm2.setTargetPosition(position);
		rm1.setTargetPosition(position);
		rm2.setTargetPosition(position);
	}

	/**
	 * Gets the angle in degrees of the robot.
	 *
	 * @return Returns the angle in degrees of the robot.
	 */
	public double getAngle() {
		// We experimentally determined the Y axis is the axis we want to use for heading angle.
		// We have to process the angle because the imu works in euler angles so the Y axis is
		// returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
		// 180 degrees. We detect this transition and track the total cumulative angle of rotation.

		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//		opMode.telemetry.addData("deltaAngle", deltaAngle);

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

	public  void encoderDriveForward(double speed, double inches) {

		int targetPos = (int)(inches * Constants.Drivetrain.TICKS_PER_INCH);

		setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(targetPos);

		drive(Math.abs(speed), 0, 0);

		while (lm1.isBusy() && lm2.isBusy() && rm1.isBusy() && rm2.isBusy()) {
//			opMode.telemetry.addData("LM1", lm1.getCurrentPosition());
//			opMode.telemetry.addData("LM2", lm2.getCurrentPosition());
//			opMode.telemetry.addData("RM1", rm1.getCurrentPosition());
//			opMode.telemetry.addData("RM2", rm2.getCurrentPosition());
//			opMode.telemetry.addData("LM1", lm1.());
//			opMode.telemetry.addData("LM2", lm2.getTargetPosition());
//			opMode.telemetry.addData("RM1", rm1.getTargetPosition());
//			opMode.telemetry.addData("RM2", rm2.getTargetPosition());
//			opMode.telemetry.update();
		}

		drive(0, 0, 0);
	}

	/**
	 * Drives the robot forward.
	 *
	 * @param distance          The distance the robot will travel in inches.
	 * @param maxSpeed          The maxSpeed the robot will travel in motor power (0-1).
	 * @param speedUpPercentage The percentage the distance will be at before starting to ramp up or down.
	 */
	public void driveForward(double distance, double maxSpeed, double speedUpPercentage, double slowDownPercentage, double timeout) {
//		opMode.telemetry.addData("Status", "Driving for " + distance + " inches");
//		opMode.telemetry.update();

		speedUpPercentage = Math.abs(speedUpPercentage);
		slowDownPercentage = Math.abs(slowDownPercentage);
		maxSpeed = Math.abs(maxSpeed);
		if (distance == 0) return;
		boolean forwards = distance > 0;
		distance *= Constants.Drivetrain.TICKS_PER_INCH; // Sets it to be the distance in ticks
		distance = Math.abs(distance);

		setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
		setTargetPosition((int) distance * (forwards ? 1 : -1));

		double percentage;
		double power;
		Timer timer = new Timer();
		timer.start();
		while (Math.abs(getDrivePosition()) < distance && !this.opMode.gamepad1.a && !timer.hasElapsed(timeout)) {
//			opMode.telemetry.addData("LM1", lm1.getCurrentPosition());
//			opMode.telemetry.addData("LM2", lm2.getCurrentPosition());
//			opMode.telemetry.addData("RM1", rm1.getCurrentPosition());
//			opMode.telemetry.addData("RM2", rm2.getCurrentPosition());
//			opMode.telemetry.addData("LM1", lm1.());
//			opMode.telemetry.addData("LM2", lm2.getTargetPosition());
//			opMode.telemetry.addData("RM1", rm1.getTargetPosition());
//			opMode.telemetry.addData("RM2", rm2.getTargetPosition());
//			opMode.telemetry.update();
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
				drive(power, 0, 0);
			}
		}
		setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
//		opMode.telemetry.addData("Status", "Turning to " + degrees + "°");
//		opMode.telemetry.update();

		setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		PIDController pidController = new PIDController(Constants.Drivetrain.TURNING_PID_KP, Constants.Drivetrain.TURNING_PID_KI, Constants.Drivetrain.TURNING_PID_KD);
		pidController.setTolerance(2);

		double targetAngle = degrees;

		Timer timer = new Timer();
		timer.start();

		loop:
		while (true) {

			double pidOutput = pidController.calculate(getAngle(), targetAngle);
			double output = pidOutput + (Math.signum(pidOutput) * Constants.Drivetrain.TURNING_PID_KF * (pidController.atSetpoint() ? 0 : 1));
//			double output = Constants.Drivetrain.TURNING_PID_KF;
			drive(0, 0, MathUtils.clamp(-output, -maxPower, maxPower));

//			opMode.telemetry.addData("Power", output);
//			opMode.telemetry.addData("Angle", getAngle());
//			opMode.telemetry.update();

			if (pidController.atSetpoint() || timer.hasElapsed(timeoutSeconds)) {
				drive(0, 0, 0);
				break loop;
			}
		}
	}

	public void turnWithEncoder(double distance, double maxSpeed, double speedUpPercentage, double slowDownPercentage, double timeout) {
//		opMode.telemetry.addData("Status", "Turning for " + (distance * 50) + " units");
//		opMode.telemetry.update();

		speedUpPercentage = Math.abs(speedUpPercentage);
		slowDownPercentage = Math.abs(slowDownPercentage);
		maxSpeed = Math.abs(maxSpeed);
		if (distance == 0) return;
		boolean forwards = distance > 0;
//		distance *= Constants.Drivetrain.TICKS_PER_INCH; // Sets it to be the distance in ticks
		distance = Math.abs(distance * 50);

		setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//		setTargetPosition((int) distance);
		lm1.setTargetPosition((int) -distance * (forwards ? 1 : -1));
		lm2.setTargetPosition((int) -distance * (forwards ? 1 : -1));
		rm1.setTargetPosition((int) distance * (forwards ? 1 : -1));
		rm2.setTargetPosition((int) distance * (forwards ? 1 : -1));

		double percentage;
		double power;
		Timer timer = new Timer();
		timer.start();
		while (Math.abs(getRightDrivePosition()) < Math.abs(distance) && !timer.hasElapsed(timeout)) {
//			opMode.telemetry.addData("LM1", lm1.getCurrentPosition());
//			opMode.telemetry.addData("LM2", lm2.getCurrentPosition());
//			opMode.telemetry.addData("RM1", rm1.getCurrentPosition());
//			opMode.telemetry.addData("RM2", rm2.getCurrentPosition());
//			opMode.telemetry.update();
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
				drive(0, 0, power);
			} else {
				// Drive backwards
				drive(0, 0, power);
			}
		}
		drive(0, 0, 0);
	}

	public double getLeftDrivePosition() {
		return (lm1.getCurrentPosition() + lm2.getCurrentPosition()) / 2;
	}

	public double getRightDrivePosition() {
		return (rm1.getCurrentPosition() + rm2.getCurrentPosition()) / 2;
	}

}
