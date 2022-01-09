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
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MecanumDriveOpMode extends LinearOpMode {
    private static final double BALL_INTAKE_POSITION = 0.50;
    private static final double CUBE_INTAKE_POSITION = 0.625;
    private static final double DROPOFF_POSITION = 0.43;
    private static final double OBSTACLE_POSITION = 0.27;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");


        DcMotor carousel_spin_blue = hardwareMap.get(DcMotor.class, "carousel_spin_blue");
        carousel_spin_blue.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor carousel_spin_red = hardwareMap.get(DcMotor.class, "carousel_spin_red");

        CRServo intake_left = hardwareMap.get(CRServo.class, "intake_left");
        CRServo intake_right = hardwareMap.get(CRServo.class, "intake_right");

        waitForStart();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel_spin_blue.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel_spin_red.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime slide_timer = new ElapsedTime();
        boolean slide_stable = true;

        while (!isStopRequested()) {

            double speed = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Mecanum Drive
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

            double carousel_power;
            if (gamepad2.dpad_down) {
                carousel_power = 0.6;
            } else {
                carousel_power = 0;
            }

            carousel_spin_blue.setPower(carousel_power);
            carousel_spin_red.setPower(carousel_power);

            front_left.setPower(scale * front_left_power);
            front_right.setPower(scale * front_right_power);
            back_left.setPower(scale * back_left_power);
            back_right.setPower(scale * back_right_power);


            double intake_power = gamepad2.right_trigger - gamepad2.left_trigger;

            if (gamepad2.left_bumper) {
                intake_power /= 3;
            }

            intake_left.setPower(-intake_power);
            intake_right.setPower(intake_power);

            double slide_power;
            boolean slide_at_bottom = false;

            slide_power = -gamepad2.right_stick_y / 1.75;
            slide_power = Math.signum(slide_power) * Math.pow(Math.abs(slide_power), 1.5);

            if (gamepad2.dpad_up) {

                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide_stable = true;

            } else if (gamepad2.y) {

                //stage 3
                slide.setTargetPosition(1358);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;

            } else if (gamepad2.b) {

                // stage 2
                slide.setTargetPosition(803);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;

            } else if (gamepad2.x||gamepad1.a) {

                //stage 1
                slide.setTargetPosition(380);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.35);
                slide_stable = true;

             }else if (gamepad2.a) {

                //bottom
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slide.setPower(0.45);
                slide_stable = true;

            }

//            else if (slide_power < 0 && slide.getCurrentPosition() < 0) {
//                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                slide_power = 0;
//                slide_at_bottom = true;

//            }
            //slide slowmode
            if (gamepad2.right_bumper) {

                slide_power *= 0.3;

            }
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

            int slide_ticks = slide.getCurrentPosition();
            telemetry.addData("Slide Encoder:", String.format("%d", slide_ticks));
            telemetry.update();




        }
    }
}