
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime
        ;


import org.firstinspires.ftc.teamcode.Util.PracticeHardware;

import static android.R.attr.mode;


@Autonomous(name="Blue Side Ver 2.4", group="Pushbot")
public class PracticeAutoRed extends LinearOpMode {
    ColorSensor colorSensor;


    PracticeHardware robot = new PracticeHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.25;
    static final double Servo = 0.8;
    static final double Servobback = -.8;
    double arm = -.2;
    private  int ticksPerRev = 1120/2;
    private int diameter = 4;

    static final double TURN_SPEED = 0.5;
    private double getTicksPerMm() {

        return ticksPerRev/(Math.PI*(diameter*25.4));
    }
    private void setMotorMode(DcMotor.RunMode mode) {

        robot.leftDrive.setMode(mode);
        robot.leftbackDrive.setMode(mode);
        robot.rightDrive.setMode(mode);
        robot.rightbackDrive.setMode(mode);
    }
    public void turn(int angle, double power){
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double tickPerDegree = 21.251/2;
        int ticks = (int)(tickPerDegree*angle);
        setEncoderTargets(ticks,-ticks,ticks,-ticks,power);
    }

    public void strafe(int dist, double power){
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        double tickPerMm = getTicksPerMm();
        int ticks = (int)(tickPerMm*dist);
        setEncoderTargets(ticks,-ticks,-ticks,ticks,power);


    }
    public void goForward(int dist, double power) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double tickPerMm = getTicksPerMm();
        int ticks = (int)(tickPerMm*dist);

        setEncoderTargets(ticks,ticks,ticks,ticks,power);
    }

    public void goBackward(int dist, double power) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double tickPerMm = getTicksPerMm();
        int ticks = (int)(tickPerMm*dist);

        setEncoderTargets(-ticks,-ticks,-ticks,-ticks,power);
    }

    public void setEncoderTargets(int fl, int fr, int bl, int br, double power) {

        double max = Math.max(Math.max(Math.abs(fr),Math.abs(fl)),Math.max(Math.abs(br),Math.abs(bl)));

        robot.leftDrive.setPower(power * Math.abs((double)fl)/max);
        robot.leftbackDrive.setPower(power * Math.abs((double)bl)/max);
        robot.rightDrive.setPower(power * Math.abs((double)fr)/max);
        robot.rightbackDrive.setPower(power * Math.abs((double)br)/max);

        robot.leftDrive.setTargetPosition(fl);
        robot.leftbackDrive.setTargetPosition(bl);
        robot.rightDrive.setTargetPosition(fr);
        robot.rightbackDrive.setTargetPosition(br);

        while(opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy() || robot.leftbackDrive.isBusy() || robot.rightbackDrive.isBusy())) {
            idle();
        }

        robot.leftDrive.setPower(0);
        robot.leftbackDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightbackDrive.setPower(0);
    }
    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && (runtime.seconds() < 4)) {
            // robot.leftDrive.setPower(-.15);
            // robot.rightDrive.setPower(-.3);
            // robot.rightbackDrive.setPower(.15);
            // robot.leftbackDrive.setPower(.3);
            robot.slideup2.setPower(.2);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarm.setPosition(Servo);
            //   robot.leftDrive.setPower(FORWARD_SPEED);
            //  robot.rightDrive.setPower(-FORWARD_SPEED);
            // robot.rightbackDrive.setPower(-FORWARD_SPEED);
            // robot.leftbackDrive.setPower(FORWARD_SPEED);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            //  robot.colorarm.setPosition(0);
            robot.colorarmx.setPower(.2);

            //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
            //  robot.right1Claw.setPosition(robot.MID_SERVO1);
            //  robot.left2Claw.setPosition(robot.MID_SERVO1);
            //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 3)) {
/*
            double fl = -.15;
            double fr = -.15;
            double br = .15;
            double bl = .15;
*/
            if (colorSensor.red() >= 1) {
                turn(20,.3);
                robot.colorarmx.setPower(-1*.2);
                robot.colorarm.setPosition(-1*.5);
                turn(-20,.3);
            }


            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());

            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
          /*
            double fl = -.15;
            double fr = -.15;
            double br = .15;
            double bl = .15;
            */
            if (colorSensor.red() < 1 && colorSensor.blue() >=2) {
                turn(-20,.3);
                robot.colorarmx.setPower(-1*.2);
                robot.colorarm.setPosition(-1*.5);
                turn(20,.3);
            }


            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());

            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        /*while (opModeIsActive() && (runtime.seconds() < 1)) {
           // robot.colorarmx.setPower(-1*.2);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightbackDrive.setPower(0);
            robot.leftbackDrive.setPower(0);

            //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
            //  robot.right1Claw.setPosition(robot.MID_SERVO1);
            //  robot.left2Claw.setPosition(robot.MID_SERVO1);
            //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarmx.setPower(0);
            robot.colorarm.setPosition(-1*.5);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightbackDrive.setPower(0);
            robot.leftbackDrive.setPower(0);
            //   robot.leftDrive.setPower(FORWARD_SPEED);
            //  robot.rightDrive.setPower(-FORWARD_SPEED);
            // robot.rightbackDrive.setPower(-FORWARD_SPEED);
            // robot.leftbackDrive.setPower(FORWARD_SPEED);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        goBackward(580,.25);
        turn(-90,.25);
        goForward(736,.25);
        turn(2,.2);
        goForward(50,.1);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robot.slideup1.setPower(-.1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robot.slideup2.setPower(-.15);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robot.slideup2.setPower(0);
            robot.left2Claw.setPosition(.5);
            robot.right2Claw.setPosition(.5);
            robot.slideup1.setPower(.15);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();


        while (opModeIsActive()) {
            idle();
        }
    }

    private void oldAuto() {

        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4)) {
            // robot.leftDrive.setPower(-.15);
            // robot.rightDrive.setPower(-.3);
            // robot.rightbackDrive.setPower(.15);
            // robot.leftbackDrive.setPower(.3);
            robot.slideup2.setPower(.15);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarm.setPosition(Servo);
            //   robot.leftDrive.setPower(FORWARD_SPEED);
            //  robot.rightDrive.setPower(-FORWARD_SPEED);
            // robot.rightbackDrive.setPower(-FORWARD_SPEED);
            // robot.leftbackDrive.setPower(FORWARD_SPEED);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            //  robot.colorarm.setPosition(0);
            robot.colorarmx.setPower(.2);

            //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
            //  robot.right1Claw.setPosition(robot.MID_SERVO1);
            //  robot.left2Claw.setPosition(robot.MID_SERVO1);
            //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 1)) {

            double fl = -.15;
            double fr = -.15;
            double br = .15;
            double bl = .15;

            if (colorSensor.red() >= 2) {
                robot.leftDrive.setPower(fl);
                robot.rightDrive.setPower(fr);
                robot.rightbackDrive.setPower(br);
                robot.leftbackDrive.setPower(bl);

                robot.applyPower(fl,fr,br,bl);
            }


            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());

            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            double fl = -.15;
            double fr = -.15;
            double br = .15;
            double bl = .15;
            if (colorSensor.red() < 1) {
                robot.leftDrive.setPower(fl);
                robot.rightDrive.setPower(fr);
                robot.rightbackDrive.setPower(br);
                robot.leftbackDrive.setPower(bl);

                robot.applyPower(fl,fr,br,bl);
            }


            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());

            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarmx.setPower(-1*.2);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightbackDrive.setPower(0);
            robot.leftbackDrive.setPower(0);

            //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
            //  robot.right1Claw.setPosition(robot.MID_SERVO1);
            //  robot.left2Claw.setPosition(robot.MID_SERVO1);
            //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarmx.setPower(0);
            robot.colorarm.setPosition(-1*.5);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightbackDrive.setPower(0);
            robot.leftbackDrive.setPower(0);
            //   robot.leftDrive.setPower(FORWARD_SPEED);
            //  robot.rightDrive.setPower(-FORWARD_SPEED);
            // robot.rightbackDrive.setPower(-FORWARD_SPEED);
            // robot.leftbackDrive.setPower(FORWARD_SPEED);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 4)) {
            double fl = -.2;
            double fr = -.2;
            double br = .2;
            double bl = .2;

            robot.leftDrive.setPower(fl);
            robot.rightDrive.setPower(fr);
            robot.rightbackDrive.setPower(br);
            robot.leftbackDrive.setPower(bl);

            robot.applyPower(fl,fr,br,bl);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
       /* robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightbackDrive.setPower(0);
        robot.leftbackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            double fl = -.2;
            double fr = -.2;
            double br = .2;
            double bl = .2;

            robot.leftDrive.setPower(fl);
            robot.rightDrive.setPower(fr);
            robot.rightbackDrive.setPower(br);
            robot.leftbackDrive.setPower(bl);

            robot.applyPower(fl,fr,br,bl);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
       */ runtime.reset();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightbackDrive.setPower(0);
        robot.leftbackDrive.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robot.slideup1.setPower(-.1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 3)) {
            robot.slideup2.setPower(-.15);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robot.slideup2.setPower(0);
            robot.left2Claw.setPosition(.5);
            robot.right2Claw.setPosition(.5);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}






