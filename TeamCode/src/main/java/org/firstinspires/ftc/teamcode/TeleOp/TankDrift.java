package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class TankDrift extends LinearOpMode {
    DcMotor left, right;
    VoltageSensor voltageSensor;

    double lp, rp = 0;
    double l_joystick, r_joystick = 0;
    double l_trigger, r_trigger = 0;
    boolean l_bumper, r_bumper = false;

    double move_power = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        double drive, turn, left, right = 0;

        initAll();

        waitForStart();

        while(opModeIsActive()){

            update();

            while(l_joystick != 0 || r_joystick != 0 ){
                update();
                drive = -l_joystick;
                turn  =  r_joystick;

                left  = drive + turn;
                right = drive - turn;

                double max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0)
                {
                    left /= max;
                    right /= max;
                }

                power(left, right);
            }

            while(l_trigger != 0 || r_trigger != 0) {
                power(l_trigger, r_trigger);
                update();
            }

            while( l_bumper || r_bumper) {
                update();
                if (r_bumper)
                    rotate(true);
                else if (l_bumper)
                    rotate(false);
            }


            while(gamepad1.dpad_up)
                power(move_power,move_power);
            while(gamepad1.dpad_down)
                power(-move_power, -move_power);

//            OPRESTE TOATE MOTOARELE DACA NIMIC NU ESTE ACTIONAT
//            OPRESTE TOT DACA APESI PE A

            if(gamepad1.a)
                stopMotor();
            if(
                    !l_bumper && !r_bumper
                    && l_joystick == 0 && r_joystick == 0
                    && l_trigger == 0 && r_trigger == 0
                    && !gamepad1.dpad_down && !gamepad1.dpad_up
            )
                stopMotor();

        }

    }

    public void initAll(){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void rotate(boolean right){
        if (right){
            this.left.setPower(1);
            this.right.setPower(-1);
        } else {
            this.left.setPower(-1);
            this.right.setPower(1);
        }
    }

    public void power(double lp, double rp){
        this.left.setPower(lp);
        this.right.setPower(rp);
    }

    public void stopMotor(){
        this.left.setPower(0);
        this.right.setPower(0);
    }

    public void update(){
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