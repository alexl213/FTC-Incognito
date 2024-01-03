package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LiftTest", group = "Linear Opmode")

public class LiftTest extends LinearOpMode {

    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //537.6 motor ticks per revolution
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //537.6 motor ticks per revolution
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Set Position Left: ", leftLift.getCurrentPosition());
            telemetry.addData("Set Position Right", rightLift.getCurrentPosition());
            telemetry.update();

            if (gamepad1.y) {
                leftLift.setTargetPosition(-1855);
                rightLift.setTargetPosition(1828);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(.4);
                rightLift.setPower(.4);
            }
            if (gamepad1.b) {
                leftLift.setTargetPosition(18);
                rightLift.setTargetPosition(0);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(.4);
                rightLift.setPower(.4);
            }
        }

    }
}
