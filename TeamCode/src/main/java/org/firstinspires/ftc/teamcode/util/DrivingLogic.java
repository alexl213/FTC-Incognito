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

    public void driveAndStrafeSlow(Gamepad gamepad1, Gamepad gamepad2, DcMotor scoringLeft, DcMotor scoringRight, double Kg) {
        while (gamepad1.left_trigger > .1) {
            drive1 = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;
            double scoring = gamepad2.right_stick_y;

            if (scoringRight.getCurrentPosition() < -20 || scoringLeft.getCurrentPosition() < -20) {//less than -20
                scoringRight.setPower(Range.clip(scoring, -.35, .12) + Kg);
                scoringLeft.setPower(Range.clip(scoring, -.35, .12) + Kg);
            } else {
                scoringRight.setPower(Range.clip(scoring, -.35, 0));
                scoringLeft.setPower(Range.clip(scoring, -.35, 0));

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
    }
}
