package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DrivingLogic {
    HardwareMap hwMap;
    Gamepad gamepad1;
    double drive1;
    double turn;
    double strafe;
    double bleftPower;
    double brightPower;
    double fleftPower;
    double frightPower;
    DcMotor bleftDrive;
    DcMotor brightDrive;
    DcMotor fleftDrive;
    DcMotor frightDrive;
    public DrivingLogic() {
    }

    public DrivingLogic(HardwareMap hwMap, Gamepad gamepad1) {
        this.hwMap = hwMap;
        this.gamepad1 = gamepad1;
    }

    public void driveMotorInit(DcMotor frightDrive, DcMotor fleftDrive, DcMotor bleftDrive, DcMotor brightDrive) {
        //method must be used after motors are initialized in main code
        this.frightDrive = frightDrive;
        this.fleftDrive = fleftDrive;
        this.bleftDrive = bleftDrive;
        this.brightDrive = brightDrive;
    }
    public void fieldCentricDriveInit(IMU imu){
        imu.resetYaw();
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));//might be UP if its asking what direction it is facing when method is called
        imu.initialize(parameters);
//        imu.resetYaw();
    }
    public void driveAndStrafeFieldCentric(Gamepad gamepad1, IMU imu, Servo scoringServoLeft, Servo scoringServoRight, Gamepad gamepad2,
                                           DcMotor scoringLeft, DcMotor scoringRight, double Kg) { //TESTINGGGGGG!
//        drive1 = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 1.5; // 1.5 = offset heading robot starts in after autonomous in RADIANS(90 degrees to left)
        //make static variable passed from auto into a separate class then into heading for offset since it can vary slightly(or not if no issues)
        double adjustedLeftX = gamepad1.left_stick_x * Math.cos(-heading) - (-gamepad1.left_stick_y) * Math.sin(-heading) ;
        double adjustedLeftY = gamepad1.left_stick_x * Math.sin(-heading) + (-gamepad1.left_stick_y) * Math.cos(-heading);

        double max = Math.max(Math.abs(adjustedLeftX) + Math.abs(adjustedLeftY) + Math.abs(turn), 1);

        fleftPower = Range.clip((adjustedLeftY + adjustedLeftX + turn) / max, -1, 1);
        frightPower = Range.clip((adjustedLeftY - adjustedLeftX - turn) / max, -1, 1);//-turn
        bleftPower = Range.clip((adjustedLeftY - adjustedLeftX + turn) / max, -1, 1);//-turn
        brightPower = Range.clip((adjustedLeftY + adjustedLeftX - turn) / max, -1, 1);

        double AdjFLP = Math.copySign(Math.pow(fleftPower, 2), fleftPower);
        double AdjFRP = Math.copySign(Math.pow(frightPower, 2), frightPower);
        double AdjBLP = Math.copySign(Math.pow(bleftPower, 2), bleftPower);
        double AdjBRP = Math.copySign(Math.pow(brightPower, 2), brightPower);

        clawOperations(scoringServoLeft, scoringServoRight,gamepad2);
        liftOperations(gamepad2, scoringLeft, scoringRight, Kg);

        bleftDrive.setPower(AdjBLP);
        brightDrive.setPower(AdjBRP);
        fleftDrive.setPower(AdjFLP);
        frightDrive.setPower(AdjFRP);
    }
    public void driveAndStrafeFieldCentricSlow(Gamepad gamepad1, Gamepad gamepad2, Servo scoringServoLeft, Servo scoringServoRight, IMU imu, DcMotor scoringLeft,
                                               DcMotor scoringRight, double Kg) { //TESTINGGGGG!
        while (gamepad1.left_trigger > .1) {
            turn = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 1.5; // 1.5 = offset heading robot starts in after autonomous in RADIANS(90 degrees to left)
            //make static variable passed from auto into a separate class then into heading for offset since it can vary slightly(or not if no issues)
            double adjustedLeftX = gamepad1.left_stick_x * Math.cos(-heading) - (-gamepad1.left_stick_y) * Math.sin(-heading) ;
            double adjustedLeftY = gamepad1.left_stick_x * Math.sin(-heading) + (-gamepad1.left_stick_y) * Math.cos(-heading);

            double max = Math.max(Math.abs(adjustedLeftX) + Math.abs(adjustedLeftY) + Math.abs(turn), 1);

            fleftPower = Range.clip((adjustedLeftY + adjustedLeftX + turn) / max, -.5, .5);
            bleftPower = Range.clip((adjustedLeftY - adjustedLeftX + turn) / max, -.5, .5);//-turn
            brightPower = Range.clip((adjustedLeftY + adjustedLeftX - turn) / max, -.5, .5);
            frightPower = Range.clip((adjustedLeftY - adjustedLeftX - turn) / max, -.5, .5);//-turn

            clawOperations(scoringServoLeft, scoringServoRight,gamepad2);
            liftOperations(gamepad2, scoringLeft, scoringRight, Kg);

            bleftDrive.setPower(bleftPower);
            brightDrive.setPower(brightPower);
            fleftDrive.setPower(fleftPower);
            frightDrive.setPower(frightPower);
        }
    }


    public void driveAndStrafe(Gamepad gamepad1) {
        drive1 = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        bleftPower = Range.clip(drive1 - strafe - turn, -1, 1);
        brightPower = Range.clip(drive1 + strafe + turn, -1, 1);
        fleftPower = Range.clip(drive1 - strafe + turn, -1, 1);
        frightPower = Range.clip(drive1 + strafe - turn, -1, 1);

        bleftDrive.setPower(bleftPower);
        brightDrive.setPower(brightPower);
        fleftDrive.setPower(fleftPower);
        frightDrive.setPower(frightPower);

    }
    public void driveAndStrafeSlowTele(Gamepad gamepad1, Gamepad gamepad2, Servo scoringServoLeft, Servo scoringServoRight) {
        while (gamepad1.left_trigger > .1) {
            drive1 = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;

            bleftPower = Range.clip(drive1 - strafe - turn, -.5, .5);
            brightPower = Range.clip(drive1 + strafe + turn, -.5, .5);
            fleftPower = Range.clip(drive1 - strafe + turn, -.5, .5);
            frightPower = Range.clip(drive1 + strafe - turn, -.5, .5);

            clawOperations(scoringServoLeft, scoringServoRight,gamepad2);

            bleftDrive.setPower(bleftPower);
            brightDrive.setPower(brightPower);
            fleftDrive.setPower(fleftPower);
            frightDrive.setPower(frightPower);
        }
    }
    public void driveAndStrafeSlow(Gamepad gamepad1) {
        while (gamepad1.left_trigger > .1) {
            drive1 = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;

                bleftPower = Range.clip(drive1 - strafe - turn, -.2, .2);
                brightPower = Range.clip(drive1 + strafe + turn, -.2, .2);
                fleftPower = Range.clip(drive1 - strafe + turn, -.2, .2);
                frightPower = Range.clip(drive1 + strafe - turn, -.2, .2);


                bleftDrive.setPower(bleftPower);
                brightDrive.setPower(brightPower);
                fleftDrive.setPower(fleftPower);
                frightDrive.setPower(frightPower);
            }
        }
        public void liftOperations(Gamepad gamepad2, DcMotor scoringLeft, DcMotor scoringRight, double Kg){

        double scoring = Math.copySign(Math.pow(gamepad2.right_stick_y, 2), gamepad2.right_stick_y);

            if (scoringRight.getCurrentPosition() < -20 || scoringLeft.getCurrentPosition() < -20) {//less than -20
                scoringRight.setPower(Range.clip(scoring, -.35, .12) + Kg);
                scoringLeft.setPower(Range.clip(scoring, -.35, .12) + Kg);
            } else {
                scoringRight.setPower(Range.clip(scoring, -.35, 0));
                scoringLeft.setPower(Range.clip(scoring, -.35, 0));
    }

}
    public void clawOperations(Servo scoringServoLeft, Servo scoringServoRight, Gamepad gamepad2){
        if (gamepad2.right_bumper) {
            scoringServoRight.setPosition(.1);//0
        }
        if (gamepad2.right_trigger > 0.2) {
            scoringServoRight.setPosition(.3);//.55
    }

        if (gamepad2.left_bumper) {
            scoringServoLeft.setPosition(.3);//.55
        }
        if (gamepad2.left_trigger > 0.2) {
            scoringServoLeft.setPosition(.1);//0
        }
    }
}
