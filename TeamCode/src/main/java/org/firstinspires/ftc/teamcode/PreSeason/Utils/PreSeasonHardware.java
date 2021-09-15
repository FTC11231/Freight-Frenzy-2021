package org.firstinspires.ftc.teamcode.PreSeason.Utils;

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
import org.firstinspires.ftc.teamcode.PreSeason.Utils.Timer;

public class PreSeasonHardware {
	/*
	 * NOTE FOR NEXT TIME I WORK ON THIS:
	 * Because EasyOpenCv depends on OpenCV-Repackaged, you will also need to copy libOpenCvNative.so
	 * from the /doc folder of that repo into the FIRST folder on the USB storage of the Robot
	 * Controller (i.e. connect the Robot Controller to your computer with a USB cable, put it into
	 * MTP mode, and drag 'n drop the file) .
	 */

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
	public PreSeasonHardware(LinearOpMode linearOpMode, Telemetry telemetry) {
		this.linearOpMode = linearOpMode;
		this.telemetry = telemetry;
	}

	/**
	 * Sets the OpMode of the robot.
	 *
	 * @param opMode    The OpMode of the robot.
	 * @param telemetry The telemetry of the robot.
	 */
	public PreSeasonHardware(OpMode opMode, Telemetry telemetry) {
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
		lm1 = this.hardwareMap.get(DcMotor.class, "LFrontDrive");
		lm2 = this.hardwareMap.get(DcMotor.class, "LRearDrive");
		rm1 = this.hardwareMap.get(DcMotor.class, "RFrontDrive");
		rm2 = this.hardwareMap.get(DcMotor.class, "RRearDrive");

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
		servo = hardwareMap.get(Servo.class, "Blocker");

		// Set servo initial position
		servo.setPosition(0);

		// Initialize sensors
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.loggingEnabled = false;
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		imu.initialize(parameters);
	}

	/**
	 * Gets the angle in degrees of the robot.
	 *
	 * @return Returns the angle in degrees of the robot.
	 */
	public double getAngle() {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); // Get the orientation of the robot

		double deltaAngle = angles.thirdAngle - lastAngles.thirdAngle; // The difference between the current angle and the angle last update

		// Wrap the angle
		if (deltaAngle < -180)
			deltaAngle += 360;
		else if (deltaAngle > 180)
			deltaAngle -= 360;

		// Set the unwrapped angle
		globalAngle += deltaAngle;
		// Set the lastAngles variable before the next update
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
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
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
	 * @param strafe Strafe.
	 * @param turn   Turn.
	 */
	public void drive(double drive, double strafe, double turn) {
		lm1.setPower(drive + strafe + turn);
		lm2.setPower(drive - strafe + turn);
		rm1.setPower(drive - strafe - turn);
		rm2.setPower(drive + strafe - turn);
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
	 * @param distance       The distance the robot will travel in inches.
	 * @param maxSpeed          The maxSpeed the robot will travel in motor power (0-1).
	 * @param rampPercentage The percentage the distance will be at before starting to ramp up or down.
	 */
	public void driveStraight(double distance, double maxSpeed, double rampPercentage) {
//        telemetry.addData("Status", "Driving for " + distance + " inches");
//        telemetry.update();
		rampPercentage = Math.abs(rampPercentage);
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
		while (getDrivePosition() < distance && this.linearOpMode.opModeIsActive() && !this.linearOpMode.gamepad1.a) {
			percentage = Math.abs(getDrivePosition() / distance);
			power = 0;
			if (forwards) {
				// Drive forwards
				if (percentage <= rampPercentage) {
					// Speed up
					power = (maxSpeed / rampPercentage) * percentage;
				} else if (percentage >= 1 - rampPercentage) {
					power = -(maxSpeed / rampPercentage) * percentage;
					// Slow down
				} else {
					// Full power
					power = maxSpeed;
				}
				MathUtils.clamp(power, 0.1, maxSpeed);
			} else {
				// Drive forwards
				if (percentage <= rampPercentage) {
					// Speed up
					power = (maxSpeed / rampPercentage) * percentage;
				} else if (percentage >= 1 - rampPercentage) {
					power = -(maxSpeed / rampPercentage) * percentage;
					// Slow down
				} else {
					// Full power
					power = maxSpeed;
				}
				MathUtils.clamp(power, -maxSpeed, 0.1);
			}
			drive(power, 0, 0);

			telemetry.addData("Percentage", percentage);
			telemetry.addData("Ramp Percentage", rampPercentage);
			telemetry.addData("Ramping Up?", percentage <= rampPercentage);

		}
	}

	/**
	 * Turns the robot to the specified angle.
	 *
	 * @param degrees         The absolute angle the robot will turn to (Forwards from the starting position is 0°, and goes counterclockwise).
	 * @param timeoutSeconds  The time the robot will turn for before stopping since the angle is close enough.
	 * @param powerMultiplier The speed the robot will go at.
	 * @return Returns the delta of the target angle and robot angle (error).
	 */
	public double turn(double degrees, double timeoutSeconds, double powerMultiplier) {
//		telemetry.addData("Status", "Turning to " + degrees + " degrees");
//		telemetry.update();

		setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

		PIDController pidController = new PIDController(org.firstinspires.ftc.teamcode.PreSeason.Utils.Constants.Drivetrain.turningPIDkP, org.firstinspires.ftc.teamcode.PreSeason.Utils.Constants.Drivetrain.turningPIDkI, org.firstinspires.ftc.teamcode.PreSeason.Utils.Constants.Drivetrain.turningPIDkD);
		pidController.setTolerance(1);

		Timer timer = new Timer();
		timer.start();

		while (this.linearOpMode.opModeIsActive()) {
			double pidOutput = pidController.calculate(getAngle(), degrees);
			double output = pidOutput + (Math.signum(pidOutput) * Constants.Drivetrain.turningPIDkF * (pidController.atSetpoint() ? 0 : 1));
			drive(0, 0, MathUtil.clamp(output, -powerMultiplier, powerMultiplier));

			if (pidController.atSetpoint() || timer.hasElapsed(timeoutSeconds)) {
				drive(0, 0, 0);
				return degrees - getAngle();
			}
		}
		return degrees - getAngle();
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
