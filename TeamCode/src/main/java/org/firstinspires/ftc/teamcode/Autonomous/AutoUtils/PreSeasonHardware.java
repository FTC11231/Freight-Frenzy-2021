package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PreSeasonHardware {

    private LinearOpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap = null;

    // Drive motors
    public DcMotor lm1 = null;
    public DcMotor lm2 = null;
    public DcMotor rm1 = null;
    public DcMotor rm2 = null;

    // Sensors
    public BNO055IMU imu = null;

    // Sensor related variables
    Orientation lastAngles = new Orientation();
    double globalAngle;

    /**
     * Sets the OpMode of the robot.
     *
     * @param opMode    The OpMode of the robot.
     * @param telemetry The telemetry of the robot.
     */
    public PreSeasonHardware(LinearOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the object with the hardware map.
     *
     * @param _hardwareMap The hardware map that the robot uses.
     */
    public void init(HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;

        // Initialize drive motors (names my change)
        lm1 = hardwareMap.get(DcMotor.class, "lm1");
        lm2 = hardwareMap.get(DcMotor.class, "lm2");
        rm1 = hardwareMap.get(DcMotor.class, "lm3");
        rm2 = hardwareMap.get(DcMotor.class, "lm4");

        // Set motor directions (since they are facing in directions, they go the wrong way)
        lm1.setDirection(DcMotorSimple.Direction.FORWARD);
        lm2.setDirection(DcMotorSimple.Direction.FORWARD);
        rm1.setDirection(DcMotorSimple.Direction.REVERSE);
        rm2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the zero power behavior of the motors to brake to stop quicker
        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotorPowers();

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
        // Set motor powers to zero to stop moving
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
        return (Math.abs(lm1.getCurrentPosition()) + Math.abs(lm2.getCurrentPosition()) + Math.abs(rm1.getCurrentPosition()) + Math.abs(rm2.getCurrentPosition()))/4;
    }

    /**
     * Drives the robot forward.
     *
     * @param distance        The distance the robot will travel in inches.
     * @param powerMultiplier The speed the robot will travel in motor power (0-1).
     * @param acceleration    The acceleration and deceleration the robot will use to drive (1+).
     */
    public void driveStraight(double distance, double powerMultiplier, double acceleration) {
        telemetry.addData("Status", "Driving for " + distance + " inches");
        telemetry.update();

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (getDrivePosition() < distance && this.opMode.opModeIsActive()) {
            double percentage = getDrivePosition() / distance;
            // Put percentage into the normal distribution formula
            // To accelerate, multiply the result by acceleration, then clamp to 0 and 1
            // To change the speed multiply by speed
            double power = powerMultiplier * MathUtil.clamp(acceleration * Math.exp(-(Math.pow(percentage - 0.5, 2) / 2 * Math.pow(0.15, 2))), 0, 1);

            drive(power, 0, 0);
        }
    }

    /**
     * Turns the robot to the specified angle.
     *
     * @param degrees         The absolute angle the robot will turn to (Forwards from the starting position is 0°, and goes counterclockwise).
     * @param timeoutSeconds  The time the robot will turn for before stopping since the angle is close enough.
     * @param powerMultiplier The speed the robot will go at.
     * @return                Returns the delta of the target angle and robot angle (error).
     */
    public double turn(double degrees, double timeoutSeconds, double powerMultiplier) {
        telemetry.addData("Status", "Turning to " + degrees + " degrees");
        telemetry.update();

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController pidController = new PIDController(Constants.Drivetrain.turningPIDkP, Constants.Drivetrain.turningPIDkI, Constants.Drivetrain.turningPIDkD);
        pidController.setTolerance(1);

        Timer timer = new Timer();
        timer.start();

        while(opMode.opModeIsActive()) {
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
     * @param seconds The delay of the OpMode
     */
    public void delay(double seconds) {
        telemetry.addData("Status", "Paused for " + seconds + "seconds");
        telemetry.update();

        Timer timer = new Timer();
        timer.start();
        while(opMode.opModeIsActive() && !timer.hasElapsed(seconds)) {}
        return;
    }

}
