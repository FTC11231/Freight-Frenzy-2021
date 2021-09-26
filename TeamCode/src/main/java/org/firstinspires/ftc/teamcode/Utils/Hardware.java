package org.firstinspires.ftc.teamcode.Utils;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {

	// Variables for output
	private LinearOpMode linearOpMode;
	private OpMode opMode;
	private Telemetry telemetry;
	private HardwareMap hardwareMap = null;

	// Drive motors
	public DcMotor lm1 = null;
	public DcMotor lm2 = null;
	public DcMotor rm1 = null;
	public DcMotor rm2 = null;

	// Servos
	public Servo servo = null;

	// Sensors
	public BNO055IMU imu = null;

	// Sensor related variables
	Orientation lastAngles = new Orientation();
	double globalAngle;

	/**
	 * Sets the LinearOpMode of the robot.
	 *
	 * @param linearOpMode The LinearOpMode of the robot.
	 * @param telemetry    The telemetry of the robot.
	 */
	public Hardware(LinearOpMode linearOpMode, Telemetry telemetry) {
		this.linearOpMode = linearOpMode;
		this.telemetry = telemetry;
	}

	/**
	 * Sets the OpMode of the robot.
	 *
	 * @param opMode    The OpMode of the robot.
	 * @param telemetry The telemetry of the robot.
	 */
	public Hardware(OpMode opMode, Telemetry telemetry) {
		this.opMode = opMode;
		this.telemetry = telemetry;
	}

	/**
	 * Initializes the object with the hardware map.
	 *
	 * @param hardwareMap The hardware map that the robot uses.
	 */
	public void init(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;

		// Initialize drive motors (names may change)
		lm1 = this.hardwareMap.get(DcMotor.class, "lm1");
		lm2 = this.hardwareMap.get(DcMotor.class, "lm2");
		rm1 = this.hardwareMap.get(DcMotor.class, "rm1");
		rm2 = this.hardwareMap.get(DcMotor.class, "rm2");

		// Set motor directions (since they are facing in directions, they go the wrong way, so we
		// reverse the right motors)
		lm1.setDirection(DcMotorSimple.Direction.REVERSE);
		lm2.setDirection(DcMotorSimple.Direction.REVERSE);
		rm1.setDirection(DcMotorSimple.Direction.FORWARD);
		rm2.setDirection(DcMotorSimple.Direction.FORWARD);

		// Set the zero power behavior of the motors to brake to stop quicker when released
		lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Set the motor powers to 0
		resetMotorPowers();

		// Initialize servos
//		servo = hardwareMap.get(Servo.class, "Blocker");

		// Set servo initial position
//		servo.setPosition(0);

		// Initialize sensors
//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		parameters.loggingEnabled = false;
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

//		imu.initialize(parameters);
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
	 * Sets the power of the motors to zero.
	 */
	public void resetMotorPowers() {
		lm1.setPower(0);
		lm2.setPower(0);
		rm1.setPower(0);
		rm2.setPower(0);
	}

	/**
	 * Drives with the parameters given.
	 *
	 * @param drive  Drive.
	 * @param turn   Turn.
	 */
	public void drive(double drive, double turn) {
		lm1.setPower(drive + turn);
		lm2.setPower(drive + turn);
		rm1.setPower(drive - turn);
		rm2.setPower(drive - turn);
	}

	/**
	 * Sets the RunMode of the drive motors.
	 *
	 * @param runMode The RunMode the motors are being set to.
	 */
	public void setRunMode(DcMotor.RunMode runMode) {
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
	public double getDrivePosition() {
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
	public void driveStraight(double distance, double maxSpeed, double speedUpPercentage, double slowDownPercentage) {
		telemetry.addData("Status", "Driving for " + distance + " inches");
		telemetry.update();

		speedUpPercentage = Math.abs(speedUpPercentage);
		slowDownPercentage = Math.abs(slowDownPercentage);
		maxSpeed = Math.abs(maxSpeed);
		if (distance == 0) return;
		boolean forwards = distance > 0;
		distance *= Constants.Drivetrain.ticksPerInch; // Sets it to be the distance in ticks
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
				drive(power, 0);
			} else {
				// Drive backwards
				drive(-power, 0);
			}
		}
		drive(0, 0);
	}

	/**
	 * Turns the robot to the specified angle.
	 *
	 * @param degrees        The absolute angle the robot will turn to (Forwards from the starting position is 0°, and goes counterclockwise).
	 * @param maxPower       The speed the robot will go at.
	 * @param timeoutSeconds The time the robot will turn for before stopping since the angle is close enough.
	 * @return Returns the delta of the target angle and robot angle (error).
	 */
	public void turn(double degrees, double maxPower, double timeoutSeconds) {
		telemetry.addData("Status", "Turning to " + degrees + "°");
		telemetry.update();

		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

		PIDController pidController = new PIDController(Constants.Drivetrain.turningPIDkP, Constants.Drivetrain.turningPIDkI, Constants.Drivetrain.turningPIDkD);
		pidController.setTolerance(1);

		double targetAngle = degrees;

		Timer timer = new Timer();
		timer.start();

		while (linearOpMode.opModeIsActive()) {
			double pidOutput = pidController.calculate(getAngle(), targetAngle);
			double output = pidOutput + (Math.signum(pidOutput) * Constants.Drivetrain.turningPIDkF * (pidController.atSetpoint() ? 0 : 1));
			drive(0, MathUtils.clamp(-output, -maxPower, maxPower));

			if (pidController.atSetpoint() || timer.hasElapsed(timeoutSeconds)) {
				drive(0, 0);
				return;
			}
		}
	}

	/**
	 * Makes a delay or pause for the robot to stop moving before doing something else
	 *
	 * @param seconds The delay of the OpMode
	 */
	public void delay(double seconds) {
		telemetry.addData("Status", "Paused for " + seconds + "seconds");
		telemetry.update();

		Timer timer = new Timer();
		timer.start();
		while (linearOpMode.opModeIsActive() && !timer.hasElapsed(seconds)) {
		}
		return;
	}
}
