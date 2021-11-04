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
    private static final int MIN_HEIGHT_TICKS = 900;
    private static final double BALL_INTAKE_POSITION = 0.57;
    private static final double CUBE_INTAKE_POSITION = 0.50;
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
        CRServo left_intake = hardwareMap.get(CRServo.class, "intake_left");
        CRServo right_intake = hardwareMap.get(CRServo.class, "intake_right");
        left_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoImpl intake_pivot = hardwareMap.get(ServoImpl.class, "intake_pivot");
       // DcMotor carousel_spin = hardwareMap.get(DcMotor.class, "carousel_spin");


        waitForStart();


        while (!isStopRequested()){
            // Mecanum Drive
            double rotated_x = -gamepad1.left_stick_y;
            double rotated_y = gamepad1.left_stick_x;
            double speed = -rotated_y;
            double strafe = rotated_x;
            double rotate = -gamepad1.right_stick_x;
            double front_left_power = (speed+strafe+rotate);
            double front_right_power = (speed-strafe-rotate);
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
            }

            double carousel_power;
            if(gamepad2.right_bumper){
                carousel_power = 0.6;
            } else {
                carousel_power = 0;
            }

            //carousel_spin.setPower(carousel_power);

            front_left.setPower(scale*front_left_power);
            front_right.setPower(scale*front_right_power);
            back_left.setPower(scale*back_left_power);
            back_right.setPower(scale*back_right_power);

            left_intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            right_intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gamepad2.dpad_down){
                intake_pivot.setPosition(BALL_INTAKE_POSITION);
            } else if (gamepad2.dpad_right){
                intake_pivot.setPosition(CUBE_INTAKE_POSITION);
            } else if (gamepad2.dpad_left){
                intake_pivot.setPosition(DROPOFF_POSITION);
            } else if (gamepad2.dpad_up){
                intake_pivot.setPosition(OBSTACLE_POSITION);
            }

        }
    }
}