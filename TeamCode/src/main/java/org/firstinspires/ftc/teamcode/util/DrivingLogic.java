package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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



    public DrivingLogic(HardwareMap hwMap, Gamepad gamepad1){
        this.hwMap = hwMap;
        this.gamepad1 = gamepad1;
    }


    private void driveAndStrafe( ){
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
}
