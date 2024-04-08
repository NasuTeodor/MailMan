package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class exemplu extends LinearOpMode {

    double putere_teoretica = 1;
    double putere_reala;

    boolean half = false;

    DcMotor left, right;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        if(gamepad1.a)
            half = !half;

        if(half)
            putere_reala = putere_teoretica/2;
        else
            putere_reala = putere_teoretica;

        if(gamepad1.dpad_up)
            putere(putere_reala, putere_reala);
        if(gamepad1.dpad_down)
            putere(-putere_reala, -putere_reala);

    }

    public void putere(double lp, double rp){
        this.left.setPower(lp);
        this.right.setPower(rp);
    }

}
