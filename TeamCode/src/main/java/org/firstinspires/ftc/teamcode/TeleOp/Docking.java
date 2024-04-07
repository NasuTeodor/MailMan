package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.DOCK_ACTIVE;
import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.FOUND;
import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.HARD_STOP;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Docking implements Runnable {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    DcMotor left, right;

    boolean engage = false;

    public Docking(DcMotor left, DcMotor right, AprilTagProcessor aprilTag, VisionPortal visionPortal) {
        if(left == null || right == null || aprilTagProcessor == null || visionPortal == null)
            throw new NullPointerException();

        this.left = left;
        this.right = right;
        this.visionPortal = visionPortal;
        this.aprilTagProcessor = aprilTag;
    }

    @Override
    public void run() {

        boolean engaged = false;

        while(DOCK_ACTIVE && !HARD_STOP){
            AprilTagDetection detection = tagDetection();
            engaged = detection.metadata != null;
            if (engaged)
                break;
        }

        while( engaged && !HARD_STOP){
            FOUND = true;
            AprilTagDetection detection = tagDetection();
            engaged = detection.metadata != null;

            // TODO: CODE TO DOCK ROBOT TO APRIL TAG

        }

        DOCK_ACTIVE = false;
        FOUND = false;

    }

    private AprilTagDetection tagDetection(){
        AprilTagDetection detection = aprilTagProcessor.getDetections().get(0);
        if ( detection.metadata != null )
            return detection;
        return null;
    }

    private void stopVision(){
        visionPortal.stopStreaming();
    }
    private void resumeVision(){
        visionPortal.resumeStreaming();
    }

}
