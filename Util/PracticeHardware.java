package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by tarik on 12/26/2017.
 */

public class PracticeHardware
{

    public DcMotor leftDrive   = null;
    public DcMotor rightDrive  = null;
    public DcMotor rightbackDrive  = null;
    public DcMotor leftbackDrive  = null;
    public DcMotor slideup1  = null;
    public DcMotor slideup2  = null;
    public DcMotor colorarmx  = null;

    public Servo left1Claw    = null;
    public Servo right1Claw   = null;
    public Servo left2Claw    = null;
    public Servo right2Claw   = null;
    public Servo colorarm = null;
    public static final double MID_SERVO       =  0.5 ;
    public static final double GRAB1    =  0.5 ;
    public static final double GRAB2  = 0.5 ;
    public static final double GRAB3    =  0.3 ;
    public static final double GRAB4  = 0.7 ;






    HardwareMap hwMap           =  null;

    public PracticeHardware(){

    }


    public void applyPower(double fl, double fr, double br, double bl){

        double max = Math.max(Math.max(fr,fl), Math.max(br,bl));
        if (max >1){
            fr  /= max;
            fl /= max;
            br /= max;
            bl /= max;
        }

        leftDrive.setPower(fl);
        leftbackDrive.setPower(bl);
        rightDrive.setPower(fr);
        rightbackDrive.setPower(br);
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        rightbackDrive = hwMap.get(DcMotor.class, "right2_drive");
        leftbackDrive = hwMap.get(DcMotor.class, "left2_drive");
        slideup1 = hwMap.get(DcMotor.class, "slideup1");
        slideup2 = hwMap.get(DcMotor.class, "slideup2");
        colorarmx = hwMap.get(DcMotor.class, "colorarmx");



        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        slideup1.setPower(0);
        slideup2.setPower(0);
        colorarmx.setPower(0);

        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftbackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        slideup1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideup2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorarmx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        left1Claw = hwMap.servo.get("left1_hand");
        right1Claw = hwMap.servo.get("right1_hand");
        left2Claw = hwMap.servo.get("left2_hand");
        right2Claw = hwMap.servo.get("right2_hand");
        colorarm =  hwMap.servo.get("colorarm");

        left1Claw.setPosition(GRAB1);
        right1Claw.setPosition(GRAB2);
        left2Claw.setPosition(GRAB3);
        right2Claw.setPosition(GRAB4);
        colorarm.setPosition(0.0);


    }
}
