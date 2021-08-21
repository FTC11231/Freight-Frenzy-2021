package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PreSeasonHardware {

    private OpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap = null;

    public DcMotor lm1 = null;
    public DcMotor lm2 = null;
    public DcMotor rm1 = null;
    public DcMotor rm2 = null;

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

        // Initialize drive motors (names my change)
        lm1 = this.hardwareMap.get(DcMotor.class, "lm1");
        lm2 = this.hardwareMap.get(DcMotor.class, "lm2");
        rm1 = this.hardwareMap.get(DcMotor.class, "lm3");
        rm2 = this.hardwareMap.get(DcMotor.class, "lm4");

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

}
