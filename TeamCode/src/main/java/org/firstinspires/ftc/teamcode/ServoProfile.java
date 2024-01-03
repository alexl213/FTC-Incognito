package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoProfile{

    public Servo servo_left;
    public Servo servo_right;

    public MotionProfile servoProfile1;
    public ElapsedTime timer = new ElapsedTime();

    double leftPos;




    public void setServoPath(){
        //this.servo_left = servoLeft;
        //this.servo_right = servoRight;
        leftPos = servoProfile1.get(timer.seconds()).getX();
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
