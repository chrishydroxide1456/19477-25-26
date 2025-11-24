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
    private GoBildaPinpointDriver pinpoint; //use later for auto

    Limelight3A limelight;

    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void adjust() {
        LLResult llresult = limelight.getLatestResult();
        if (!llresult.getFiducialResults().isEmpty()) {
            LLResultTypes.FiducialResult fidresult = llresult.getFiducialResults().get(0);
            if (fidresult.getFiducialId() = ) {
                headingAdjust = llresult.getTx();
                //distance = llresult.
                targetVel = ;//complete this later
            } //add in red and blue alliance ids later
        }
        else {
            headingAdjust = 0.0;
            targetVel = 0.0;
        }

//
//        if (llresult.isValid() && llresult.getFiducialResults().equals(24)) { //need to add the enum stuff for red and blue
//            headingAdjust = llresult.getTx(); //add in distance later
//            targetVel = (0.1) * 30665.95/llresult.getTa(); //need to tune later. watch coach pratt vid

    }

}





