package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Base hardware mapping class for Odometry pods (dead wheels).
 * These are usually just encoder motors you DO NOT set power to.
 */
public class OdomPods {

    // Declare odometry encoders
    public DcMotor verticalLeft = null;   // left pod
    public DcMotor verticalRight = null;  // right pod
    public DcMotor horizontal = null;     // strafe pod

    /**
     * Initialize odometry encoders
     * @param hwMap FTC provided HardwareMap from your OpMode
     */
    public void init(HardwareMap hwMap) {
        // Match these names to your DS config
        verticalLeft  = hwMap.get(DcMotor.class, "leftOdo");
        verticalRight = hwMap.get(DcMotor.class, "rightOdo");
        horizontal    = hwMap.get(DcMotor.class, "backOdo");

        // Stop motors from trying to move
        verticalLeft.setPower(0);
        verticalRight.setPower(0);
        horizontal.setPower(0);

        // Reverse any directions if needed
        verticalLeft.setDirection(DcMotor.Direction.FORWARD);
        verticalRight.setDirection(DcMotor.Direction.REVERSE);
        horizontal.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run using encoder (so you can read positions)
        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Returns encoder ticks */
    public int getLeftOdoTicks() { return verticalLeft.getCurrentPosition(); }
    public int getRightOdoTicks() { return verticalRight.getCurrentPosition(); }
    public int getBackOdoTicks() { return horizontal.getCurrentPosition(); }
}



