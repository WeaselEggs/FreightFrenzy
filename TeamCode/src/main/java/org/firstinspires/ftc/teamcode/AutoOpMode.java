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
    private boolean just_duck = false;
    private static final double BALL_INTAKE_POSITION = 0.50;
    private static final double CUBE_INTAKE_POSITION = 0.625;
    private static final double DROPOFF_POSITION = 0.43;
    private static final double OBSTACLE_POSITION = 0.27;


    @Override
    public void runOpMode() throws InterruptedException {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
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

        choosewell();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime slide_timer = new ElapsedTime();
        boolean slide_stable = true;
        double slide_power;
        int slide_ticks = slide.getCurrentPosition();

        if (wait_choice == true) {

            //Waits for 10 seconds
            waitfor(10000);
        }
        if (just_duck == true) {

            //Strafe left toward carousel
            drive(.55,0,0,500);
            drive(0, -.55, 0, 500);
            drive(-.45,0,0,300);
            drive(-.1, 0, 0, 600);

            //Spin the carousel spinners
            if (is_blue == true) {
                carousel_spin_blue.setPower(-.35);
            } else if (is_red == true) {
                carousel_spin_red.setPower(.35);
            }
            waitfor(3850);

            if (is_blue == true) {
                carousel_spin_blue.setPower(0);
            } else if (is_red == true) {
                carousel_spin_red.setPower(0);
            }

            //Parks in Depot
            drive(.55,0,0,550);
            drive(0,-.55,0,150);


        } else if (skip_duck == true) {

            //Slide kit to stage 3
            slide.setTargetPosition(1364);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.35);
            slide_stable = true;

            waitfor(1000);

            //Go forward until the alliance shipping hub
            drive(.55,0, 0, 750);


            //Out-take the cube
            drive(.1,0,0,100);
            waitfor(100);
            intake_left.setPower(.5);
            intake_right.setPower(-.5);
            waitfor(800);

            //Stop out-take
            intake_left.setPower(0);
            intake_right.setPower(0);

            //Slide kit to stage 1
            slide.setTargetPosition(380);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.35);
            slide_stable = true;

            //Strafe and rotate
            drive(-.55,0,0,200);
            if (is_red == true) {
                drive(0,0,.5,750);
            } else if (is_blue == true) {
                drive(0,0,-.5,750);
            }


            //Strafe right and parks in the warehouse
            drive(.55, 0, 0 , 3700);

            //Slide kit to bottom
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.45);
            slide_stable = true;

            waitfor(2000);


        } else if (just_park == false) {
            //Strafe left toward carousel
            drive(.55,0,0,500);
            drive(0, -.55, 0, 500);
            drive(-.45,0,0,300);
            drive(-.1, 0, 0, 600);

            //Spin the carousel spinners
            if (is_blue == true) {
                carousel_spin_blue.setPower(-.35);
            } else if (is_red == true) {
                carousel_spin_red.setPower(.35);
            }
            waitfor(4000);

            if (is_blue == true) {
                carousel_spin_blue.setPower(0);
            } else if (is_red == true) {
                carousel_spin_red.setPower(0);
            }

            //Go forward until the alliance shipping hub
            drive(.55,0, 0, 300);

            //Slide kit to stage 3
            slide.setTargetPosition(1364);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.35);
            slide_stable = true;

            //Strafe right and forward toward the alliance shipping hub
            drive(0,.55,0,2150);
            drive(.55,0, 0, 300);


            //Out-take the cube
            drive(.1,0,0,100);
            waitfor(100);
            intake_left.setPower(.5);
            intake_right.setPower(-.5);
            waitfor(800);

            //Stop out-take
            intake_left.setPower(0);
            intake_right.setPower(0);


            //Strafe and rotate
            drive(-.55,0,0,200);
            if (is_red == true) {
                drive(0,0,.5,750);
            } else if (is_blue == true) {
                drive(0,0,-.5,750);
            }

            //Slide kit to stage 1
            slide.setTargetPosition(380);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.35);
            slide_stable = true;


            //Strafe right and parks in the warehouse
            drive(.55, 0, 0 , 3700);

            //Slide kit to bottom
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setPower(0.45);
            slide_stable = true;

            waitfor(2000);
          

        } else {

            //Goes and parks into the warehouse
            drive(.55,0, 0, 600);
            drive(.55, 0, 0 , 3700);
        }

        slide_power = -gamepad2.right_stick_y / 1.75;
        slide_power = Math.signum(slide_power) * Math.pow(Math.abs(slide_power), 1.5);

        if (Math.abs(slide_power) < 0.05) {
            if (!slide_stable) {
                if (slide_timer.milliseconds() < 200) {
                    slide.setPower(0.05);
                } else {
                    slide.setTargetPosition(slide.getCurrentPosition());
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    slide.setPower(0.35);
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
            if(gamepad1.right_bumper) {
                just_duck = true;
            }
            if(gamepad1.left_bumper) {
                just_duck = false;
            }


            telemetry.addData("Alliance Color Red(dpad left)", is_red ? "yes" : "no");
            telemetry.addData("Alliance Color Blue(dpad right)", is_blue ? "yes" : "no");
            telemetry.addData("Wait(a/b)", wait_choice ? "yes" : "no");
            telemetry.addData("Just Park(x/y)", just_park ? "yes" : "no");
            telemetry.addData("Skip duck(dpad up/dpad down)", skip_duck ? "yes" : "no");
            telemetry.addData("Just duck(right bumper/left bumper)", just_duck ? "yes" : "no");
            telemetry.update();
        }
    }


    private void drive(double speed, double strafe, double rotate, long milis){

        if (is_blue == true) {
            strafe = -strafe;
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
