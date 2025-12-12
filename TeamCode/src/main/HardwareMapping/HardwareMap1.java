package org.firstinspires.ftc.teamcode.HardwareMapping;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Base hardware mapping class for motors.
 * Add motors here as you wire them on your robot.
 */
public class HardwareMap1 {

    // Declare motors
    public DcMotor FLW = null;
    public DcMotor FRW = null;
    public DcMotor BLW = null;
    public DcMotor BRW = null;

    public DcMotor InM = null;
    //public Servo InPWM;

    public DcMotorEx ShotM  = null;
    public DcMotorEx ShotRotateM = null;
    public DcMotorEx RevM = null;

    public Servo TransSFL = null;
    public Servo TransSFR = null;
    public Servo TransSBL = null;
    public Servo TransSBR = null;
    public Servo BallLiftS = null;
    public Servo PitchS = null;

    //public Servo CamS = null;
// Sensors
    public RevColorSensorV3 ColS0 = null;
    public RevColorSensorV3 ColS120 = null;
    public RevColorSensorV3 ColS240 = null;

    public Limelight3A limelight = null;  // Assuming Limelight connected via USB as webcam

    // --- Odometry Encoders --- FIX THIS, WE NEED STILL FOR AUTO
    public DcMotor leftOdo = null;
    public DcMotor rightOdo = null;
    public DcMotor perpOdo = null;



    /**
     * Initialize hardware devices
     * @param hwMap FTC provided HardwareMap from your OpMode
     */
    public void init(HardwareMap hwMap) {
        // Map motor names to configuration names from the Driver Station config
        FLW  = hwMap.get(DcMotorEx.class, "FLW");
        FRW = hwMap.get(DcMotorEx.class, "FRW");
        BLW   = hwMap.get(DcMotorEx.class, "BLW");
        BRW  = hwMap.get(DcMotorEx.class, "BRW");
        ShotM  = hwMap.get(DcMotorEx.class, "ShotM");

        InM = hwMap.get(DcMotor.class, "InM");
        //InPWM = hwMap.get(Servo.class, "InPWM");
//        InPWM.setPower(0.0);

        ShotRotateM = hwMap.get(DcMotorEx.class, "ShotRotateM");
        RevM = hwMap.get(DcMotorEx.class, "RevM");

        TransSFL = hwMap.get(Servo.class, "TransSFL");
        TransSFR = hwMap.get(Servo.class, "TransSFR");
        TransSBL = hwMap.get(Servo.class, "TransSBL");
        TransSBR = hwMap.get(Servo.class, "TransSBR");
        BallLiftS = hwMap.get(Servo.class, "BallLiftS");
        PitchS = hwMap.get(Servo.class, "PitchS");
        ColS0 = hwMap.get(RevColorSensorV3.class, "ColS0");
        ColS120 = hwMap.get(RevColorSensorV3.class, "ColS120");
        ColS240 = hwMap.get(RevColorSensorV3.class, "ColS240");
        //limelight = hwMap.get(Limelight3A .class, "limelight");

        // --- Odometry Encoders ---
//        leftOdo = hwMap.get(DcMotor.class, "leftOdo");
//        rightOdo = hwMap.get(DcMotor.class, "rightOdo");
//        perpOdo = hwMap.get(DcMotor.class, "perpOdo");
//        // Reset odometry encoders
//        leftOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //leftOdo
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //rightOdo
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //PerpOdo

        RevM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set motor directions (flip as needed)
        //FLW.setDirection(DcMotor.Direction.FORWARD);
        //BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setDirection(DcMotor.Direction.FORWARD);

        InM.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        FLW.setPower(0);
        FRW.setPower(0);
        BLW.setPower(0);
        BRW.setPower(0);

        // Set all motors to run without encoders by default
        //FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //InM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShotRotateM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShotM.setDirection(DcMotor.Direction.REVERSE);
        ShotM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ShotM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShotM.setPower(0.0);
        ShotRotateM.setDirection(DcMotor.Direction.REVERSE);
// Initialize pitch servo

    }
}