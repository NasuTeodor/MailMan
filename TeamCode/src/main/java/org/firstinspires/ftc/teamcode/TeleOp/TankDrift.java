package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.ThreadHandler.*;

import android.widget.TableRow;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class TankDrift extends LinearOpMode {
    DcMotor left, right, arm;
    VoltageSensor voltageSensor;

    Servo gheara;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;
    FocusControl focusControl;
    ExposureControl exposureControl;

    Docking docking;
    Thread dockerThread;

    double l_joystick, r_joystick;
    double l_trigger, r_trigger;
    boolean l_bumper, r_bumper;

    double move_power = 1;
    public static double open = 1;
    public static double close = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        double drive, turn, left, right;

        initAll();

        update();

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

            if (gamepad1.x)
                THREAD_SHOULD_CLOSE = true;

            if (gamepad1.y)
                DOCK_ACTIVE = !DOCK_ACTIVE;

            if (gamepad2.x) {
                gheara.setPosition(open);
            } else if (gamepad2.y) {
                gheara.setPosition(close);
            }

            update();

            if (!DOCK_FOUND) {

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
                    move_arm(l_trigger, r_trigger);
                    update();
                } else
                    arm.setPower(0);

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

            if (isStopRequested()) {
                THREAD_SHOULD_CLOSE = true;
                dockerThread.join(10);
            }

        }
        dockerThread.join(0);
    }

    public void initAll() {
        THREAD_SHOULD_CLOSE = false;

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        arm = hardwareMap.get(DcMotor.class, "arm");

        gheara = hardwareMap.get(Servo.class, "servus");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        docking = new Docking(this.left, this.right, this.arm, this.aprilTagProcessor, this.visionPortal);
        dockerThread = new Thread(docking);
        dockerThread.start();

//        aprilTagProcessor = new AprilTagProcessor.Builder()
////                .setDrawCubeProjection(true)
////                .setDrawTagID(true)
////                .setLensIntrinsics(3553.31878217, 3553.31878217, 2341.71664461, 1689.28387843)
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//        aprilTagProcessor.setDecimation(1); // lower fps

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(BuiltinCameraDirection.BACK);
////        builder.setCameraResolution(new Size(1920, 1080));
//        builder.enableLiveView(true);
//        builder.addProcessor(aprilTagProcessor);
//        visionPortal = builder.build();
    }

    public void move_arm(double l_trigger, double r_trigger) {
        double left = Math.abs(l_trigger);
        double right = Math.abs(r_trigger);
        double power = right - left;
        arm.setPower(power);
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
        telemetry.addData("DOCKING ACTIVE", DOCK_ACTIVE);
        if (dockerThread != null)
            telemetry.addData("THREAD", dockerThread.getState());
        telemetry.update();

        this.l_joystick = gamepad1.left_stick_y;
        this.r_joystick = gamepad1.right_stick_x;

        this.l_trigger = gamepad2.left_trigger;
        this.r_trigger = gamepad2.right_trigger;

        this.l_bumper = gamepad1.left_bumper;
        this.r_bumper = gamepad1.right_bumper;
    }

}
