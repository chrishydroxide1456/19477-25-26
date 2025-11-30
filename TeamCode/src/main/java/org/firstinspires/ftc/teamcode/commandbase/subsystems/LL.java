package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.subsystems.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();
    private LL() {}

    // Shared values used by other subsystems
    public static double targetVel;
    public static double headingAdjust; // degrees
    public static float distance;       // inches (smoothed)
    public static int ID;

    private Limelight3A limelight;

    // TUNE THESE TO YOUR ROBOT (measure once, then leave alone)
    private static final double LIMELIGHT_HEIGHT_IN = 14.5;  // camera lens height off floor
    private static final double GOAL_HEIGHT_IN = 47.0;       // target center height off floor
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0; // camera up-angle

    // Smooth distance tracking
    private float smoothedDistance = 36.0f;  // start with reasonable guess
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT_MS = 500; // 0.5s timeout

    public void initialize(HardwareMap hardwareMap) {
        ID = 0;
        headingAdjust = 0.0;
        distance = 36.0f;
        targetVel = 0.0;
        smoothedDistance = 36.0f;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setID() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && !llresult.getFiducialResults().isEmpty()) {
            ID = llresult.getFiducialResults().get(0).getFiducialId();
        }
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        long currentTime = System.currentTimeMillis();

        if (llresult != null && !llresult.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fidresult = llresult.getFiducialResults().get(0);
            lastDetectionTime = currentTime;

            // Horizontal aiming (always responsive)
            headingAdjust = llresult.getTx();

            // Raw distance calculation
            double ty = llresult.getTy();
            double angleToGoalDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
            double angleToGoalRad = Math.toRadians(angleToGoalDeg);

            float rawDistance = 36.0f; // default
            if (Math.abs(Math.tan(angleToGoalRad)) > 1e-6) {
                double distInches = (GOAL_HEIGHT_IN - LIMELIGHT_HEIGHT_IN) / Math.tan(angleToGoalRad);
                rawDistance = (float) Math.max(12.0, Math.min(120.0, distInches)); // clamp 1-10ft
            }

            // Smooth the distance (prevents jumping)
            smoothedDistance = smoothDistance(rawDistance, smoothedDistance);
            distance = smoothedDistance;

            // Velocity based on smoothed distance
            double calculatedVel = gettargetVel(distance);
            targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                    ? calculatedVel : 0.0;

        } else {
            // No detection: coast distance for a bit, then give up
            if (currentTime - lastDetectionTime < DETECTION_TIMEOUT_MS) {
                // Still coasting on last good distance
                distance = smoothedDistance;
                headingAdjust = 0.0; // stop turning when tag lost
                targetVel = gettargetVel(distance); // keep flywheels spinning briefly
            } else {
                // Totally lost tag
                headingAdjust = 0.0;
                targetVel = 0.0;
                distance = 0.0f;
                smoothedDistance = 36.0f; // reset to default
            }
        }
    }

    // Exponential moving average smoothing
    private float smoothDistance(float newDistance, float oldDistance) {
        final double alpha = 0.3; // 30% new, 70% old (tune 0.1-0.5)
        return (float) (alpha * newDistance + (1.0 - alpha) * oldDistance);
    }

    public double gettargetVel(float Distance) {
        double Hgoal = 47;
        double Hshooter = 14.5;
        double launchangle = 50 * Math.PI / 180.0;
        double flywheelR = 2;

        double denom = Distance * Math.tan(launchangle) - (Hgoal - Hshooter);
        if (denom <= 0.1) denom = 0.1;

        double Vball = Math.sqrt((9.8 * Distance * Distance) /
                (2 * Math.cos(launchangle) * denom));
        double Vwheel = Vball / 0.8;
        return Vwheel / (2 * Math.PI * flywheelR);
    }
}
