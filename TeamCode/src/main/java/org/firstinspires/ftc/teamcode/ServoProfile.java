package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.DrivingLogic;

public class ServoProfile{

    public Servo servo_left;
    public Servo servo_right;

    public MotionProfile servoProfile1;
    public ElapsedTime timer = new ElapsedTime();


    double leftPos;




    public void setServoPath( double intakePower, double scoringleftPower,
                             double scoringrightPower, DcMotor bleftDrive, DcMotor brightDrive, DcMotor fleftDrive, DcMotor frightDrive, double intake1, double scoring, Gamepad gamepad1, Gamepad gamepad2, DrivingLogic robot){

        leftPos = servoProfile1.get(timer.seconds()).getX();


        intake1 = gamepad2.left_stick_x;
        scoring = gamepad2.right_stick_y;
        intakePower = Range.clip(intake1, -.45, .45);
        scoringleftPower = Range.clip(scoring, -0.65, 0.1);
        scoringrightPower = Range.clip(scoring, -0.65, 0.1);


        robot.driveAndStrafe(gamepad1);
        //robot.driveAndStrafeSlow(gamepad1,gamepad2, );cannot drive slow while moving arm up or down
        setPositionsSynced(leftPos);



    }
    public void generateProfile(double maxVelo, double maxAccel,double startPosition, double endPosition){
        servoProfile1 = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(startPosition, 0, 0),new MotionState(endPosition, 0,0), maxVelo, maxAccel);
        timer.reset();
    }
    public void initServos( Servo servoLeft, Servo servoRight){
        this.servo_left = servoLeft;
        this.servo_right = servoRight;
        //this.servo_left = hwmap.get(Servo.class, servoLeft);
       // this.servo_left = hwmap.get(Servo.class, servoRight);
    }
    public void setPositionsSynced(double leftPos){
        servo_left.setPosition(leftPos);
        servo_right.setPosition(1-leftPos);//.91

    }



}
