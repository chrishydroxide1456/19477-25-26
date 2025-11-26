package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOp.drive;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import kotlin.jvm.internal.Lambda;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
public class LL implements Subsystem {

    public static final LL INSTANCE = new LL();
    private LL() {}
    public static double targetVel; //calculate when needed using Atag info
    public static double headingAdjust;
    public static double distance;
    private GoBildaPinpointDriver pinpoint; //use later for auto
    public static int ID;

    Limelight3A limelight;

    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setID() {
        LLResult llresult = limelight.getLatestResult(); //concerns with motif apriltags
        if (!llresult.getFiducialResults().isEmpty()) {
            ID = llresult.getFiducialResults().get(0).getFiducialId();
        }
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        if (!llresult.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fidresult = llresult.getFiducialResults().get(0); //fiducialresult index is determined by Ta, should be fine
            if (fidresult.getFiducialId() == ID) {
                headingAdjust = llresult.getTx();
                distance = (29.502 - cameraheight)/(Math.tan(cammountangle + llresult.getTy())); //verify 29.502 = apriltag center height, need to find cameraheight and cammountangle later. also, adjust so that it is the point of release rather than actual LL location
                targetVel = gettargetVel(distance); //i hope this works
            }
        }
        else {
            headingAdjust = 0.0;
            targetVel = 0.0;
        }

    }

    public double gettargetVel(double Distance) {
        double Hgoal = ;
        double Hshooter = ;
        double launchangle = ;
        double flywheelR = ;

        double Vball = Math.sqrt((9.8*Distance*Distance)/(2*Math.cos(launchangle)*(Distance*Math.tan(launchangle)-(Hgoal-Hshooter))));
        double Vwheel = Vball/ ;
        return (Vwheel/(2*Math.PI*flywheelR));
    }

}





