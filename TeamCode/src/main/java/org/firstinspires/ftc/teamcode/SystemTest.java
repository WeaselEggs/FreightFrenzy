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
public class SystemTest extends LinearOpMode {
    //private static final double BALL_INTAKE_POSITION = 0.50;
    //private static final double CUBE_INTAKE_POSITION = 0.625;
    //private static final double DROPOFF_POSITION = 0.43;
    //private static final double OBSTACLE_POSITION = 0.27;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");

        //CRServo carousel_spin_blue = hardwareMap.get(CRServo.class, "carousel_spin_blue");

       // CRServo carousel_spin_red = hardwareMap.get(CRServo.class, "carousel_spin_red");
        Servo capper = hardwareMap.get(Servo.class, "scissor");
        float capper_position= 0;
        Servo flipper = hardwareMap.get(Servo.class,"tilt" );
        float flipper_position = 0;


        //CRServo intake_spin = hardwareMap.get(CRServo.class, "intake_spin");
        //intake_pivot.setPosition(CUBE_INTAKE_POSITION);

        waitForStart();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boolean old_left_bumper1=false;
        boolean old_right_bumper1=false;
        boolean old_left_bumper2=false;
        boolean old_right_bumper2=false;
        while (!isStopRequested()){
/*
            double rotated_x = -gamepad1.left_stick_y;
            double rotated_y = gamepad1.left_stick_x;
            double speed = -rotated_y;
            double strafe = rotated_x;

            // Mecanum Drive
            double rotate = -gamepad1.right_stick_x;
            double front_left_power = (speed-strafe-rotate);
            double front_right_power = (speed+strafe+rotate);
            double back_left_power = (speed-strafe+rotate);
            double back_right_power = (speed+strafe-rotate);
            double max = Math.max(Math.max(Math.abs(front_left_power), Math.abs(front_right_power)),
                    Math.max(Math.abs(back_left_power), Math.abs(back_right_power)));
            double scale;
            if (max>1){
                scale = 1/max;
            } else{
                scale = 1;
            }
            if (gamepad1.left_bumper) {
                scale /= 3;
            }*/

            //double carousel_power;
         //   if(gamepad2.right_bumper){
             //   carousel_power = 0.6;
           // } else {
          //      carousel_power = 0;
//            }

           // carousel_spin_blue.setPower(carousel_power);
          //  carousel_spin_red.setPower(carousel_power);

            front_left.setPower(gamepad1.left_stick_x);
            front_right.setPower(gamepad1.right_stick_x);
            back_left.setPower(gamepad1.left_stick_y);
            back_right.setPower(gamepad1.right_stick_y);


            //old intake system. delete once we dismantle pivot intake.

           /* double intake_power = gamepad2.right_trigger - gamepad2.left_trigger;

            if (gamepad2.left_bumper) {
                intake_power /= 3;
            }
            intake_spin.setPower(intake_power); */

            double slide_power;
            slide.setPower(-gamepad2.right_stick_y);
            int slide_ticks = slide.getCurrentPosition();

            if (gamepad2.a) {
                slide.setTargetPosition(1380);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x) {
                slide.setTargetPosition(870);
            }
            if (gamepad1.b) {
                slide.setTargetPosition(350);
            }

            if(gamepad1.left_bumper && !old_left_bumper1){
                capper_position += .05;
            }
            if(gamepad1.right_bumper&& !old_right_bumper1){
                capper_position -= .05;
            }
            old_left_bumper1= gamepad1.left_bumper;
            old_right_bumper1=gamepad1.right_bumper;
            capper.setPosition(capper_position);

            if(gamepad2.left_bumper && !old_left_bumper2){
                flipper_position += .05;
            }
            if(gamepad2.right_bumper&& !old_right_bumper2){
                flipper_position -= .05;
            }
            old_left_bumper2= gamepad2.left_bumper;
            old_right_bumper2=gamepad2.right_bumper;
            flipper.setPosition(flipper_position);

            telemetry.addData("Slide Encoder:", String.format("%d", slide_ticks));

            telemetry.addData("capper position", String.format("%.2f", capper_position));
            telemetry.addData("flipper position", String.format("%.2f", flipper_position));

           /*
           if (gamepad2.dpad_down){
                intake_pivot.setPosition(BALL_INTAKE_POSITION);
            } else if (gamepad2.dpad_right){
                intake_pivot.setPosition(CUBE_INTAKE_POSITION);
            } else if (gamepad2.dpad_left){
                intake_pivot.setPosition(DROPOFF_POSITION);
            } else if (gamepad2.dpad_up){
                intake_pivot.setPosition(OBSTACLE_POSITION);

           }
           */




            telemetry.update();

        }
    }
}