package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoOpMode extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private boolean is_blue = false;
    private boolean is_red = true;
    private boolean wait_choice = false;
    private boolean just_park = false;
    private boolean skip_duck = false;
    private static final double BALL_INTAKE_POSITION = 0.50;
    private static final double CUBE_INTAKE_POSITION = 0.625;
    private static final double DROPOFF_POSITION = 0.43;
    private static final double OBSTACLE_POSITION = 0.27;


    @Override
    public void runOpMode() throws InterruptedException {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor carousel_spin_blue = hardwareMap.get(DcMotor.class, "carousel_spin_blue");
        carousel_spin_blue.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor carousel_spin_red = hardwareMap.get(DcMotor.class, "carousel_spin_red");
        CRServo left_intake = hardwareMap.get(CRServo.class, "intake_left");
        CRServo right_intake = hardwareMap.get(CRServo.class, "intake_right");
        left_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoImpl intake_pivot = hardwareMap.get(ServoImpl.class, "intake_pivot");

        intake_pivot.setPosition(OBSTACLE_POSITION);

        choosewell();

        if (wait_choice == true) {

            //Waits for 10 seconds
            waitfor(10000);
        }

        if (skip_duck == true) {

            //Go forward until the alliance shipping hub
            drive(0,-.55, 0, 750);

            //Out-take the cube
            intake_pivot.setPosition(DROPOFF_POSITION);
            waitfor(500);
            left_intake.setPower(-.5);
            right_intake.setPower(-.5);
            waitfor(500);
            intake_pivot.setPosition(OBSTACLE_POSITION);
            waitfor(500);

            //Parks into the warehouse
            drive(.55, 0, 0 , 2300);



        } else if (just_park == false) {
            // Go forward until the alliance shipping hub
            drive(0, -.55, 0, 750);

            // Change intake position and out-take the cube
            intake_pivot.setPosition(DROPOFF_POSITION);
            waitfor(500);
            left_intake.setPower(-.5);
            right_intake.setPower(-.5);
            waitfor(500);
            intake_pivot.setPosition(OBSTACLE_POSITION);
            waitfor(500);

            // Strafe right until the perimeter
            drive(-.55, 0, 0, 1500);
            left_intake.setPower(0);
            right_intake.setPower(0);

            // Go "back" until the carousel (into the corner)
            drive(0, .35, 0, 950);
            drive(0, .1, 0, 600);

            // Spin the carousel
            if (is_blue == true) {
                carousel_spin_blue.setPower(.35);
            } else if (is_red == true) {
                carousel_spin_red.setPower(.35);
            }
            waitfor(2800);

            // Back out of the corner to line up with the warehouse
            if (is_blue == true) {
                carousel_spin_blue.setPower(0);
            } else if (is_red == true) {
                carousel_spin_red.setPower(0);
            }
            drive(0, -.55, 0, 600);

            // Strafe left to get into the warehouse
            drive(.55, 0, 0, 4000);
        } else {

            //Goes and parks into the warehouse
            drive(0, -.55, 0, 500);
            drive(.55, 0, 0, 2300);
        }
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
            if(gamepad1.dpad_up) {
              skip_duck = true;
            }
            if(gamepad1.dpad_down) {
                skip_duck = false;
            }
            if (gamepad1.a) {
                wait_choice = true;
            }
            if(gamepad1.b) {
                wait_choice = false;
            }
            if(gamepad1.x) {
                just_park = true;
            }
            if(gamepad1.y) {
                just_park = false;
            }


            telemetry.addData("Alliance Color Red(dpad left)", is_red ? "yes" : "no");
            telemetry.addData("Alliance Color Blue(dpad right)", is_blue ? "yes" : "no");
            telemetry.addData("Wait(a/b)", wait_choice ? "yes" : "no");
            telemetry.addData("Just Park(x/y)", just_park ? "yes" : "no");
            telemetry.addData("Skip duck(dpad up/dpad down)", skip_duck ? "yes" : "no");
            telemetry.update();
        }
    }

    private void drive(double speed, double strafe, double rotate, long milis){

        strafe = -strafe;
        if (is_red == true) {
            speed = -speed;
        }

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
        front_left.setPower(scale*front_left_power);
        front_right.setPower(scale*front_right_power);
        back_left.setPower(scale*back_left_power);
        back_right.setPower(scale*back_right_power);

        waitfor(milis);

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    private void waitfor(long milis){
        ElapsedTime timer=new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds()<milis){
            idle();
        }
    }


}
