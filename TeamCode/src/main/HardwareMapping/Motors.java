package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Base hardware mapping class for motors.
 * Add motors here as you wire them on your robot.
 */
public class Motors {

    // Declare motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    public DcMotor intakeMotor = null;

    public DcMotor shooterMotor = null;

    /**
     * Initialize hardware devices
     * @param hwMap FTC provided HardwareMap from your OpMode
     */
    public void init(HardwareMap hwMap) {
        // Map motor names to configuration names from the Driver Station config
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        shooterMotor   = hwMap.get(DcMotor.class, "shooterMotor");
        intakeMotor  = hwMap.get(DcMotor.class, "intakeMotor");

        // Set motor directions (flip as needed)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders by default
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
