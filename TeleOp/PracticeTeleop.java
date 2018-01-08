package org.firstinspires.ftc.teamcode.TeleOp;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Util.PracticeHardware;

/*
@Author Tarik C. Brown
 */
@TeleOp(name="Teleop Ver 2.4", group="Pushbot")
public class PracticeTeleop extends OpMode {

    double wristOffset = .5;
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = .05 ;

    PracticeHardware robot       = new PracticeHardware();
    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double fl;
        double fr;
        double br;
        double bl;

        double power = .6*-gamepad1.left_stick_y;
        double rotation = .6*gamepad1.left_stick_x;
        double strafe = .6*-gamepad1.right_stick_x;

        double up1 =  -gamepad2.right_stick_x;
        double up2 =  gamepad2.right_stick_y;

        fl = power + strafe + rotation;
        fr = power - strafe - rotation;
        br = power + strafe - rotation;
        bl = power - strafe + rotation;



       /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
       double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
       double rightX = gamepad1.right_stick_x;
       final double v1 = r * Math.cos(robotAngle) + rightX; // power(left y) + strafe(left x) + rotation(right x)
       final double v2 = r * Math.sin(robotAngle) - rightX; // power - strafe - rotation
       final double v3 = r * Math.sin(robotAngle) + rightX; // power - strafe +rotation
       final double v4 = r * Math.cos(robotAngle) - rightX; // p + s - r
*/



       robot.leftDrive.setPower(fl);
       robot.rightDrive.setPower(fr);
       robot.leftbackDrive.setPower(bl);
       robot.rightbackDrive.setPower(br);
        robot.colorarmx.setPower(0);


        robot.applyPower(fl, fr, br, bl);

        robot.slideup1.setPower(up1);
        robot.slideup2.setPower(up2);

        if (gamepad2.left_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.right_bumper)
            clawOffset -= CLAW_SPEED;

        if (gamepad2.left_stick_button)
            wristOffset += CLAW_SPEED;
        else if (gamepad2.right_stick_button)
            wristOffset -= CLAW_SPEED;




        // Move both servos to new position.  Assume servos are mirror image of each other.
        //   clawOffset = Range.clip(clawOffset, 0, 1);
        // wristOffset = Range.clip(clawOffset, 0, 1);
        robot.left1Claw.setPosition(robot.MID_SERVO + clawOffset);
        robot.right1Claw.setPosition(robot.MID_SERVO - clawOffset);
        robot.left2Claw.setPosition(robot.MID_SERVO + wristOffset);
        robot.right2Claw.setPosition(robot.MID_SERVO - wristOffset);




     /*  double left;
       double right;
       double up1;
       double up2;
       double grab1;



         // Tank Drive
       left = .25*gamepad1.left_stick_y;
       right = .25*-gamepad1.right_stick_y;
           // Lift Motors
       up1 = -.2*gamepad2.left_stick_y;
       up2 = .2*gamepad2.right_stick_y;
       grab1 = .08*gamepad2.right_stick_x;

       robot.leftDrive.setPower(left);
       robot.rightDrive.setPower(right);
       robot.leftbackDrive.setPower(left);
       robot.rightbackDrive.setPower(right);
       robot.slideup1.setPower(up1);
       robot.slideup2.setPower(up2);
       robot.grab.setPower(grab1);
*/
        telemetry.addData("left",  "%.2f", fl);
        telemetry.addData("right", "%.2f", fr);
        telemetry.addData("right2", "%.2f", br);
        telemetry.addData("left2", "%.2f", bl);



    }

    @Override
    public void stop() {
    }
}

