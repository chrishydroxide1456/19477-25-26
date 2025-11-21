package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();
    private LL() {}
    
    // Target velocity calculation constant
    // This constant is derived from the relationship between Limelight target area (ta)
    // and desired shooter velocity. Formula: velocity = (VELOCITY_SCALE_FACTOR * DISTANCE_CONSTANT) / ta
    // TODO: Tune this value based on field testing - see Coach Pratt's video for calibration method
    private static final double DISTANCE_CONSTANT = 30665.95;
    
    public static double targetVel; //calculate when needed using AprilTag info
    public static double headingAdjust;
    private GoBildaPinpointDriver pinpoint;
    double distance;

    Limelight3A limelight;

    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            LLResultTypes.FiducialResult fiducialresult = llresult.getFiducialResults();
            // Check if we have fiducial results and if the target ID is 24
            if (fiducialresult != null && !fiducialresult.getTargets().isEmpty()) {
                // TODO: Check for specific fiducial ID (24) - need to iterate through targets
                headingAdjust = llresult.getTx();
                
                // Prevent division by zero
                double ta = llresult.getTa();
                if (ta > 0.0) {
                    targetVel = (0.1) * DISTANCE_CONSTANT / ta;
                } else {
                    targetVel = 0.0;
                }
            } else {
                headingAdjust = 0.0;
                targetVel = 0.0;
            }
        } else {
            headingAdjust = 0.0;
            targetVel = 0.0;
        }
    }

}



