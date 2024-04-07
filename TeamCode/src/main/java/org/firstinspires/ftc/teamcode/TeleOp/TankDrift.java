package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TankDrift extends LinearOpMode {
    DcMotor left, right;
    VoltageSensor voltageSensor;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    FocusControl focusControl;
    ExposureControl exposureControl;

    Thread arm_rise, arm_lower, arm_stop;

    double lp, rp = 0;
    double l_joystick, r_joystick = 0;
    double l_trigger, r_trigger = 0;
    boolean l_bumper, r_bumper = false;

    double move_power = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        double drive, turn, left, right;

        initAll();

//        // EXPERIMENTAL
//        APARENT "GETTING CONTROLS IS ONLY SUPPORTED FOR WEBCAMS"
//        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            int a = 0;
//        }
//        focusControl = visionPortal.getCameraControl(FocusControl.class);
//        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//
//        focusControl.setMode(FocusControl.Mode.ContinuousAuto);
//        exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);

        waitForStart();

        while (opModeIsActive()) {

            update();

            if (!FOUND) {

                if (l_joystick != 0 || r_joystick != 0) {
                    update();
                    drive = -l_joystick;
                    turn = r_joystick;

                    left = drive + turn;
                    right = drive - turn;

                    double max = Math.max(Math.abs(left), Math.abs(right));
                    if (max > 1.0) {
                        left /= max;
                        right /= max;
                    }

                    power(left, right);
                }

                if (l_trigger != 0 || r_trigger != 0) {
                    power(l_trigger, r_trigger);
                    update();
                }

                if (l_bumper || r_bumper) {
                    update();
                    if (r_bumper)
                        rotate(true);
                    else if (l_bumper)
                        rotate(false);
                }


                if (gamepad1.dpad_up)
                    power(move_power, move_power);
                if (gamepad1.dpad_down)
                    power(-move_power, -move_power);

            }

//          OPRESTE TOATE MOTOARELE DACA NIMIC NU ESTE ACTIONAT
//          OPRESTE TOT DACA APESI PE A
            while (gamepad1.a)
                stopMotor();
            if (!l_bumper && !r_bumper
                    && l_joystick == 0 && r_joystick == 0
                    && l_trigger == 0 && r_trigger == 0
                    && !gamepad1.dpad_down && !gamepad1.dpad_up)
                stopMotor();

        }

    }

    public void initAll() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setLensIntrinsics(3553.31878217, 3553.31878217, 2341.71664461, 1689.28387843)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        aprilTagProcessor.setDecimation(1); // lower fps

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(BuiltinCameraDirection.BACK);
//        builder.setCameraResolution(new Size(1920, 1080));
        builder.enableLiveView(true);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public void rotate(boolean right) {
        if (right) {
            this.left.setPower(1);
            this.right.setPower(-1);
        } else {
            this.left.setPower(-1);
            this.right.setPower(1);
        }
    }

    public void power(double lp, double rp) {
        this.left.setPower(lp);
        this.right.setPower(rp);
    }

    public void stopMotor() {
        this.left.setPower(0);
        this.right.setPower(0);
    }

    public void update() {
        telemetry.addData("V", voltageSensor.getVoltage());
        telemetry.update();

        this.l_joystick = gamepad1.left_stick_y;
        this.r_joystick = gamepad1.right_stick_x;

        this.l_trigger = gamepad1.left_trigger;
        this.r_trigger = gamepad1.right_trigger;

        this.l_bumper = gamepad1.left_bumper;
        this.r_bumper = gamepad1.right_bumper;
    }

}
