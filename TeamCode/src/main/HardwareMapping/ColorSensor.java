package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;


public class ColorSensor {

    public RevColorSensorV3 ColS0 = null;
    public RevColorSensorV3 ColS120 = null;
    public RevColorSensorV3 ColS240 = null;

    public void init(HardwareMap hwMap) {
        ColS0 = hwMap.get(RevColorSensorV3.class, "ColS0");
        ColS120 = hwMap.get(RevColorSensorV3.class, "ColS120");
        ColS240 = hwMap.get(RevColorSensorV3.class, "ColS240");

    }

    /**
     * Returns normalized RGB values (0.0–1.0)
     */
    public double[] getNormalizedRGB(RevColorSensorV3 sensor) {
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        double total = r + g + b;
        if (total == 0) {
            return new double[]{0,0,0};
        }

        return new double[]{ r / total, g / total, b / total };
    }

    // ----------------------------
// HSV-based color detection
// ----------------------------
    public String detectColorHSV(RevColorSensorV3 sensor) {
        float[] hsv = getHSV(sensor);
        float hue = hsv[0];        // 0–360 degrees
        float sat = hsv[1];        // 0–1
        float val = hsv[2];        // 0–1

        // If too dark → unreliable
        if (val < 0.05) return "Unknown";

        // ---- TUNED HSV RANGES ----
        boolean isGreen =
                (hue > 30 && hue < 178) && sat > 0.05;        // Greenish area

        boolean isPurple =
                ((hue > 182 && hue < 320) && sat > 0.05);     // Purple area

        if (isGreen) return "Green";
        if (isPurple) return "Purple";

        return "Unknown";
    }


    /**
     * Only detect Green or Purple.
     * All other values return "Unknown".
     */
    public String detectColorS0() {
        if (ColS0 == null) return "Unknown";

        double[] rgb = getNormalizedRGB(ColS0);
        double r = rgb[0];
        double g = rgb[1];
        double b = rgb[2];

        // ---- Tuned thresholds ----
        boolean isGreen =
                g > 0.44 && r < 0.235;
        boolean isPurple =
                r > 0.24 && b > 0.33;

        if (isGreen) return "Green";
        if (isPurple) return "Purple";

        return "Unknown";
    }
    public String detectColorS120() {
        if (ColS120 == null) return "Unknown";

        double[] rgb = getNormalizedRGB(ColS120);
        double r = rgb[0];
        double g = rgb[1];
        double b = rgb[2];

        // ---- Tuned thresholds ----
        boolean isGreen =
                g > 0.44 && r < 0.235;
        boolean isPurple =
                r > 0.24 && b > 0.33;

        if (isGreen) return "Green";
        if (isPurple) return "Purple";

        return "Unknown";
    }
    public String detectColorS240() {
        if (ColS240 == null) return "Unknown";

        double[] rgb = getNormalizedRGB(ColS240);
        double r = rgb[0];
        double g = rgb[1];
        double b = rgb[2];

        // ---- Tuned thresholds ----
        boolean isGreen =
                g > 0.44 && r < 0.235;
        boolean isPurple =
                r > 0.24 && b > 0.33;

        if (isGreen) return "Green";
        if (isPurple) return "Purple";

        return "Unknown";
    }

    public float[] getHSV(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        // hsv[0] = hue (0–360)
        // hsv[1] = saturation (0–1)
        // hsv[2] = value (0–1)
        return hsv;
    }
    public String detectColorHSV_S0() {
        if (ColS0 == null) return "Unknown";
        return detectColorHSV(ColS0);
    }

    public String detectColorHSV_S120() {
        if (ColS120 == null) return "Unknown";
        return detectColorHSV(ColS120);
    }

    public String detectColorHSV_S240() {
        if (ColS240 == null) return "Unknown";
        return detectColorHSV(ColS240);
    }

    // ----------------------------
    // Distance sensing
    // ----------------------------
    public double getDistanceCM(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public double getDistanceMM(RevColorSensorV3 sensor) {
        return sensor.getDistance(DistanceUnit.MM);
    }
    //double distanceThresholdCM = 2;
    public String detectColorWithDistance(RevColorSensorV3 sensor, double distanceThresholdCM) {
        if (sensor == null) return "Unknown";

        double dist = sensor.getDistance(DistanceUnit.CM);

        // Filter out invalid or noisy readings
        if (Double.isNaN(dist) || dist <= 0 || dist > distanceThresholdCM) {
            return "None";
        }

        return detectColorHSV(sensor);
    }



    public boolean isGreen_S0(double distanceThresholdCM) {
        return detectColorWithDistance(ColS0, distanceThresholdCM).equals("Green");
    }

    public boolean isPurple_S0(double distanceThresholdCM) {
        return detectColorWithDistance(ColS0, distanceThresholdCM).equals("Purple");
    }

    public boolean isGreen_S120(double distanceThresholdCM) {
        return detectColorWithDistance(ColS120, distanceThresholdCM).equals("Green");
    }

    public boolean isPurple_S120(double distanceThresholdCM) {
        return detectColorWithDistance(ColS120, distanceThresholdCM).equals("Purple");
    }

    public boolean isGreen_S240(double distanceThresholdCM) {
        return detectColorWithDistance(ColS240, distanceThresholdCM).equals("Green");
    }

    public boolean isPurple_S240(double distanceThresholdCM) {
        return detectColorWithDistance(ColS240, distanceThresholdCM).equals("Purple");
    }

}
