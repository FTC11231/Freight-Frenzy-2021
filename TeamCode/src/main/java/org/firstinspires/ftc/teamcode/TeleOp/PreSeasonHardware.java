package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PreSeasonHardware {

    public DcMotor lm1 = null;
    public DcMotor lm2 = null;
    public DcMotor rm1 = null;
    public DcMotor rm2 = null;

    private HardwareMap hwMap = null;

    public void init(HardwareMap _hwMap) {
        hwMap = _hwMap;

        // Initialize drive motors (names my change)
        lm1 = hwMap.get(DcMotor.class, "lm1");
        lm2 = hwMap.get(DcMotor.class, "lm2");
        rm1 = hwMap.get(DcMotor.class, "lm3");
        rm2 = hwMap.get(DcMotor.class, "lm4");

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

        resetMotorSpeeds();
    }

    public void resetMotorSpeeds() {
        lm1.setPower(0);
        lm2.setPower(0);
        rm1.setPower(0);
        rm2.setPower(0);
    }

}
