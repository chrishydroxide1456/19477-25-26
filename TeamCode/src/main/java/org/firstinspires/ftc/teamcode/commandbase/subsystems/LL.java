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
    public static double headingAdjust;
    public static float distance;
    public static int ID;  // Set this in teleop to filter tags!
    public static boolean tagVisible = false;

    private Limelight3A limelight;

    // Camera mount constants
    private static final double LIMELIGHT_HEIGHT_IN = 14.5;
    private static final double GOAL_HEIGHT_IN = 47.0;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    // Distance smoothing
    private static final double SMOOTHING_ALPHA = 0.5;
    private float smoothedDistance = 36.0f;
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT_MS = 500;

    public void initialize(HardwareMap hardwareMap) {
        headingAdjust = 0.0;
        distance = 36.0f;
        targetVel = 0.0;
        smoothedDistance = 36.0f;
        tagVisible = false;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    @Override
    public void periodic() {
        adjust();
    }

    public void setID() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.getFiducialResults() != null && !llresult.getFiducialResults().isEmpty()) {
            ID = llresult.getFiducialResults().get(0).getFiducialId();
        }
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        long currentTime = System.currentTimeMillis();

        if (llresult != null && llresult.getFiducialResults() != null && !llresult.getFiducialResults().isEmpty()) {

            // **KEY CHANGE**: Find the tag that matches our target ID
            LLResultTypes.FiducialResult targetTag = null;
            for (LLResultTypes.FiducialResult fidResult : llresult.getFiducialResults()) {
                if (fidResult.getFiducialId() == ID) {
                    targetTag = fidResult;
                    break;
                }
            }

            // Only process if we found our specific tag
            if (targetTag != null) {
                tagVisible = true;
                lastDetectionTime = currentTime;

                // Horizontal aiming
                headingAdjust = llresult.getTx();

                // Raw distance calculation
                double ty = llresult.getTy();
                double angleToGoalDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
                double angleToGoalRad = Math.toRadians(angleToGoalDeg);

                float rawDistance = 36.0f;
                if (Math.abs(Math.tan(angleToGoalRad)) > 1e-6) {
                    double distInches = (GOAL_HEIGHT_IN - LIMELIGHT_HEIGHT_IN) / Math.tan(angleToGoalRad);
                    if (!Double.isNaN(distInches) && !Double.isInfinite(distInches) && distInches > 0) {
                        rawDistance = (float) Math.max(12.0, Math.min(120.0, distInches));
                    }
                }

                // Smooth distance
                smoothedDistance = smoothDistance(rawDistance, smoothedDistance);
                distance = smoothedDistance;

                // Velocity based on smoothed distance
                double calculatedVel = gettargetVel(distance);
                targetVel = (!Double.isNaN(calculatedVel) && !Double.isInfinite(calculatedVel))
                        ? calculatedVel : 800.0;

            } else {
                // Camera sees tags, but NOT our target ID - treat as no detection
                handleNoDetection(currentTime);
            }

        } else {
            // No tags visible at all
            handleNoDetection(currentTime);
        }
    }

    private void handleNoDetection(long currentTime) {
        if (currentTime - lastDetectionTime < DETECTION_TIMEOUT_MS) {
            // Coasting on last good measurement
            tagVisible = true;
            distance = smoothedDistance;
            headingAdjust = 0.0;
            targetVel = gettargetVel(distance);
        } else {
            // Tag actually lost
            tagVisible = false;
            headingAdjust = 0.0;
            targetVel = 0.0;
            distance = 0.0f;
            smoothedDistance = 36.0f;
        }
    }

    private float smoothDistance(float newDistance, float oldDistance) {
        return (float) (SMOOTHING_ALPHA * newDistance + (1.0 - SMOOTHING_ALPHA) * oldDistance);
    }

    // CORRECT: Ballistic physics with proper unit conversions
    public double gettargetVel(double Distance) {
        // Convert everything to meters
        double Hshooter = LIMELIGHT_HEIGHT_IN / 39.37;  // inches → meters
        double Hgoal = GOAL_HEIGHT_IN / 39.37;           // inches → meters
        double launchangle = Math.toRadians(50);
        double flywheelR = 2.0 / 39.37;                  // 2 inches → meters

        double DistanceMeters = Distance / 39.37;
        double heightDiff = Hgoal - Hshooter;

        double cosAngle = Math.cos(launchangle);
        double tanAngle = Math.tan(launchangle);
        double denom = DistanceMeters * tanAngle - heightDiff;

        // Safety check for invalid trajectories
        if (denom <= 0.05) {
            denom = 0.05;
        }

        // Ballistic calculation
        double numerator = 9.8 * DistanceMeters * DistanceMeters;
        double Vball = Math.sqrt(numerator / (2.0 * cosAngle * cosAngle * denom));

        // Account for energy loss (flywheel to ball efficiency)
        double Vwheel = Vball / 0.55;

        // Convert linear velocity (m/s) to RPM
        double rpm = (Vwheel / (2.0 * Math.PI * flywheelR)) * 60.0;

        // Clamp to motor limits
        return Math.max(800, Math.min(2800, rpm));
    }
}