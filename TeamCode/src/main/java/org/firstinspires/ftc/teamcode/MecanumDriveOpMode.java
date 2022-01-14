package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MecanumDriveOpMode extends LinearOpMode {
    private static final double CAPPER_STORED_POSITION = .8;
    private static final double CAPPER_TRAVEL_POSITION = .65;
    private static final double CAPPER_CAPPING_POSITION = .4;
    private static final double CAPPER_PICKUP_POSITION =.25;

    private static final int SLIDE_BOTTOM_TICKS = 0;
    private static final int SLIDE_LEVEL1_TICKS = 380;
    private static final int SLIDE_LEVEL2_TICKS = 803;
    private static final int SLIDE_LEVEL3_TICKS = 1358;
    private static final int SLIDE_CAPPER_HOVER_TICKS = 1635;
    private static final int SLIDE_CAPPER_CAPPING_TICKS = 1280;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");

        DcMotor carousel_spin_blue = hardwareMap.get(DcMotor.class, "carousel_spin_blue");
        DcMotor carousel_spin_red = hardwareMap.get(DcMotor.class, "carousel_spin_red");
        carousel_spin_red.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo intake_left = hardwareMap.get(CRServo.class, "intake_left");
        CRServo intake_right = hardwareMap.get(CRServo.class, "intake_right");

        Servo capper = hardwareMap.get(Servo.class, "capper");
        capper.setPosition(CAPPER_STORED_POSITION);

        waitForStart();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel_spin_blue.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel_spin_red.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime slide_timer = new ElapsedTime();
        boolean slide_stable = true;

        while (!isStopRequested()) {
            // DRIVE: Mecanum Drive
            double speed = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double front_left_power = (speed + strafe + rotate);
            double front_right_power = (speed - strafe - rotate);
            double back_left_power = (speed - strafe + rotate);
            double back_right_power = (speed + strafe - rotate);
            double max = Math.max(Math.max(Math.abs(front_left_power), Math.abs(front_right_power)),
                    Math.max(Math.abs(back_left_power), Math.abs(back_right_power)));
            double scale;
            if (max > 1) {
                scale = 1 / max;
            } else {
                scale = 1;
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                scale /= 3;
            }
            front_left.setPower(scale * front_left_power);
            front_right.setPower(scale * front_right_power);
            back_left.setPower(scale * back_left_power);
            back_right.setPower(scale * back_right_power);

            // CAROUSEL
            double carousel_power;
            if (gamepad2.dpad_down||gamepad1.left_trigger>0.5) {
                carousel_power = 0.6;
            } else {
                carousel_power = 0;
            }
            carousel_spin_blue.setPower(carousel_power);
            carousel_spin_red.setPower(carousel_power);

            // INTAKE
            double intake_power = gamepad2.left_trigger - gamepad2.right_trigger;
            if (gamepad2.left_bumper) {
                intake_power /= 3;
            }
            intake_left.setPower(-intake_power);
            intake_right.setPower(intake_power);

            // SLIDE: move slides to specific positions
            if (gamepad2.dpad_up) {
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide_stable = true;
            } else if (gamepad2.y) { // move slide to level 3
                slide.setTargetPosition(SLIDE_LEVEL3_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;
            } else if (gamepad2.b) { // move slide to level 2
                slide.setTargetPosition(SLIDE_LEVEL2_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;
            } else if (gamepad2.x||gamepad1.a) { // move slide to level 1
                slide.setTargetPosition(SLIDE_LEVEL1_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;
             } else if (gamepad2.a) { // move slide to bottom
                slide.setTargetPosition(SLIDE_BOTTOM_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.45);
                slide_stable = true;
            } else if (gamepad2.dpad_left) { // move slide to capping hover position
                slide.setTargetPosition(SLIDE_CAPPER_HOVER_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;
            } else if (gamepad2.dpad_right) { // move slide to capping drop position
                slide.setTargetPosition(SLIDE_CAPPER_CAPPING_TICKS);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;
            }

            // SLIDE: slide manual mode
            double slide_power = -gamepad2.right_stick_y / 1.75;
            slide_power = Math.signum(slide_power) * Math.pow(Math.abs(slide_power), 1.5);
            // slide slowmode
            if (gamepad2.right_bumper) {
                slide_power *= 0.3;
            }

            // SLIDE: hold slide in place when we reach target position
            if (Math.abs(slide_power) < 0.05) {
                if (!slide_stable) {
                    if (slide_timer.milliseconds() < 200) {
                        slide.setPower(0.05);
                    } else {
                        slide.setTargetPosition(slide.getCurrentPosition());
                        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        slide.setPower(0.08);
                        slide_stable = true;
                    }
                }
            } else {
                if (slide_stable) {
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide_stable = false;
                }

                slide_timer.reset();
                slide.setPower(slide_power);
            }

            // CAPPER
            if(gamepad1.dpad_up){
                capper.setPosition(CAPPER_TRAVEL_POSITION);
            }
            if(gamepad1.dpad_right){
                capper.setPosition(CAPPER_CAPPING_POSITION);
            }
            if(gamepad1.dpad_left){
                capper.setPosition(CAPPER_STORED_POSITION);
            }
            if (gamepad1.dpad_down) {
                capper.setPosition(CAPPER_PICKUP_POSITION);
            }

            int slide_ticks = slide.getCurrentPosition();
            telemetry.addData("Slide Encoder:", String.format("%d", slide_ticks));
            telemetry.update();
        } // while
    }
}
