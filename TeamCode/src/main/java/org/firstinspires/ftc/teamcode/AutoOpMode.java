package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoOpMode extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private boolean is_blue;
    private boolean is_red;
    private boolean fancy_auto;
    private boolean wait_choice;


    @Override
    public void runOpMode() throws InterruptedException {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        choosewell();
        waitForStart();
    }

    private void choosewell() {
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_left) {
                is_blue = false;
                is_red = true;
            }
            if (gamepad1.dpad_right) {
                is_red = false;
                is_blue = true;
            }
            if (gamepad1.a) {
                fancy_auto = true;
            }
            if (gamepad1.b) {
                fancy_auto = false;
            }
            telemetry.addData("fancy auto(a/b)", fancy_auto ? "yes" : "no");
            telemetry.addData("Alliance Color Red(dpad left/dpad right)", is_red ? "yes" : "no");
            telemetry.addData("Alliance Color Blue(dpad up/dpad down)", is_blue ? "yes" : "no");
            telemetry.addData("wait(x/y)", wait_choice ? "true" : "false");
            telemetry.update();
        }
    }


}
