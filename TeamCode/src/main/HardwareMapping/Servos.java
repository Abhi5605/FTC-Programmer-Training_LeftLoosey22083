package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Base hardware mapping class for Servos
 * Add all your robot servos here
 * Add MORE NOW AS WE DON"T KNOW WHICH ONES ARE NEEDED/
 * public class Servos
 */
public class Servos {

    // Declare servos
    //public Servo leftClaw = null;
    //public Servo rightClaw = null;
    public Servo clutch = null;

    /**
     * Initialize servos
     * @param hwMap FTC provided HardwareMap from your OpMode
     */
    public void init(HardwareMap hwMap) {
        clutch   = hwMap.get(Servo.class, "clutchServo");

        // Set initial positions (0 to 1)
        clutch.setPosition(0);
    }


    public void clutch(double v) {
    }
}
