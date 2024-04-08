package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.ARM_PRESS;
import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.DOCK_ACTIVE;
import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.DOCK_FOUND;
import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.THREAD_SHOULD_CLOSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Docking implements Runnable {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    DcMotor left, right, arm;

    boolean engage = false;

    public Docking(DcMotor left, DcMotor right, DcMotor arm, AprilTagProcessor aprilTag, VisionPortal visionPortal) {
//        if(left == null || right == null || aprilTagProcessor == null || visionPortal == null)
//            throw new NullPointerException();

        this.left = left;
        this.right = right;
        this.arm = arm;
        this.visionPortal = visionPortal;
        this.aprilTagProcessor = aprilTag;
    }

    @Override
    public void run() {
        while (!THREAD_SHOULD_CLOSE) {

            boolean engaged = false;
            boolean test = true;
            double drive, turn, left, right;

        while(DOCK_ACTIVE){
            AprilTagDetection detection = tagDetection();
            engaged = detection.metadata != null;
            if (engaged)
                break;
        }

        while( engaged){
            DOCK_FOUND = true;
            AprilTagDetection detection = tagDetection();
            engaged = detection.metadata != null;

            // TODO: CODE TO DOCK ROBOT TO APRIL TAG

        }

        DOCK_ACTIVE = false;
        DOCK_FOUND = false;

        }
    }

    private AprilTagDetection tagDetection() {
        if(aprilTagProcessor == null)
            return null;
        AprilTagDetection detection = aprilTagProcessor.getDetections().get(0);
        if (detection.metadata != null)
            return detection;
        return null;
    }

    private void power(double lp, double rp) {
        this.left.setPower(lp);
        this.right.setPower(rp);
    }

    private void stopMotor() {
        this.left.setPower(0);
        this.right.setPower(0);
    }

    private void stopVision() {
        visionPortal.stopStreaming();
    }

    private void resumeVision() {
        visionPortal.resumeStreaming();
    }

}
