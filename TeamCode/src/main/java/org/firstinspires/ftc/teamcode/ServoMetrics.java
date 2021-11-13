package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Disabled
public class ServoMetrics extends LinearOpMode {
    private static final int MIN_HEIGHT_TICKS = 900;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo intake_pivot = hardwareMap.get(Servo.class, "intake_pivot");

        waitForStart();

        double position = .5;
        boolean lastA = false;
        boolean lastB = false;

        while (!isStopRequested()){

            intake_pivot.setPosition(position);
            if (!lastA && gamepad2.a) {
                position = position + .05;
            }
            if (!lastB && gamepad2.b){
                position = position - .05;
            }

            lastA = gamepad2.a;
            lastB = gamepad2.b;

            telemetry.addData("servo position", String.format("%1.2f", position));
            telemetry.update();

        }
    }
}
