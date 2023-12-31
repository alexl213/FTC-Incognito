package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


    @TeleOp(name="Robott", group="Linear Opmode")

    public class Driver extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor bleftDrive = null;
        private DcMotor brightDrive = null;
        private DcMotor fleftDrive = null;
        private DcMotor frightDrive = null;
        private CRServo intake = null;
        private DcMotor scoringLeft = null;
        private DcMotor scoringRight = null;
        private CRServo intakeDrop = null;
        private Servo axonLeft = null;
        private Servo axonRight = null;
        private Servo armAngle = null;
        private Servo scoringservoLeft = null;
        private Servo scoringservoRight = null;
        private DcMotor hangLeft = null;
        private DcMotor hangRight = null;

        public ServoProfile servoProfile = new ServoProfile();



        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            bleftDrive = hardwareMap.get(DcMotor.class, "bleft_Drive");
            brightDrive = hardwareMap.get(DcMotor.class, "bright_Drive");
            fleftDrive = hardwareMap.get(DcMotor.class, "fleft_Drive");
            frightDrive = hardwareMap.get(DcMotor.class, "fright_Drive");
            intake = hardwareMap.get(CRServo.class, "intake");
            scoringLeft = hardwareMap.get(DcMotor.class, "scoring_Left");
            scoringRight = hardwareMap.get(DcMotor.class, "scoring_Right");
            intakeDrop = hardwareMap.get(CRServo.class, "intakeDrop");
            axonLeft = hardwareMap.get(Servo.class, "axonLeft");
            axonRight = hardwareMap.get(Servo.class, "axonRight");
            armAngle = hardwareMap.get(Servo.class, "armAngle");
            scoringservoLeft = hardwareMap.get(Servo.class, "scoringLeft");
            scoringservoRight = hardwareMap.get(Servo.class, "scoringRight");
            hangLeft = hardwareMap.get(DcMotor.class, "hang_Left");
            hangRight = hardwareMap.get(DcMotor.class, "hang_Right");


            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            bleftDrive.setDirection(DcMotor.Direction.REVERSE);
            brightDrive.setDirection(DcMotor.Direction.FORWARD);
            fleftDrive.setDirection(DcMotor.Direction.REVERSE);
            frightDrive.setDirection(DcMotor.Direction.REVERSE);
            //intake.setDirection(DcMotor.Direction.FORWARD);
            scoringLeft.setDirection(DcMotor.Direction.FORWARD);
            scoringRight.setDirection(DcMotor.Direction.REVERSE);

            hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //537.6 motor ticks per revolution
            hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //537.6 motor ticks per revolution


            // Setup a variable for each drive wheel to save power level for telemetry
            double bleftPower;
            double brightPower;
            double fleftPower;
            double frightPower;
            double intakePower;
            double scoringleftPower;
            double scoringrightPower;
            double leftarmPos;
            double rightarmPos;
            double pos;




            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = -gamepad1.left_stick_x;
                double intake1 = gamepad2.left_stick_x;
                double scoring = gamepad2.right_stick_y;
                servoProfile.initServos(axonLeft, axonRight);

                bleftPower = Range.clip(drive - strafe - turn, -1, 1);
                brightPower = Range.clip(drive + strafe + turn, -1, 1);
                fleftPower = Range.clip(drive - strafe + turn, -1, 1);
                frightPower = Range.clip(drive + strafe - turn, -1, 1);
                intakePower = Range.clip(intake1, -.45, .45);
                scoringleftPower = Range.clip(scoring, -0.65, 0.1);
                scoringrightPower = Range.clip(scoring, -0.65, 0.1);


                while (gamepad2.a) { //zero armAngle
                    armAngle.setPosition(0);
                }
                while (gamepad2.b) {//obsolete
                    intakeDrop.setPower(-1);
                }
                intakeDrop.setPower(0);

                if (gamepad2.y){ //hanging position going up to grab on to pole
                    hangLeft.setTargetPosition(-1855);
                    hangRight.setTargetPosition(1828);
                    hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hangLeft.setPower(.4);
                    hangRight.setPower(.4);
                }

                if (gamepad2.x) { //hanging position going down to lift robot off ground
                    hangLeft.setTargetPosition(18);
                    hangRight.setTargetPosition(0);
                    hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hangLeft.setPower(.4);
                    hangRight.setPower(.4);
                }

                if (gamepad2.dpad_up) {
                    armAngle.setPosition(0);
                    runtime.reset();
                    servoProfile.generateProfile(.34, .23, .3, .9);
                     while (servoProfile.servoProfile1.get(runtime.seconds()).getX() <= .9 && opModeIsActive()) {
                         bleftPower = Range.clip(drive - strafe - turn, -1, 1);
                         brightPower = Range.clip(drive + strafe + turn, -1, 1);
                         fleftPower = Range.clip(drive - strafe + turn, -1, 1);
                         frightPower = Range.clip(drive + strafe - turn, -1, 1);
                         intakePower = Range.clip(intake1, -.45, .45);
                         scoringleftPower = Range.clip(scoring, -0.65, 0.1);
                         scoringrightPower = Range.clip(scoring, -0.65, 0.1);
                         servoProfile.setServoPath();

                     }
                    armAngle.setPosition(.50);

                     //armAngle.setPosition(0);
                    // if(runtime.seconds() == .6){
                    //sleep(600);
                     //axonRight.setPosition(.015);
                     //axonLeft.setPosition(.9);
                    // }
                     //if(runtime.seconds() == 1.6){
                    //sleep(1000);
                     //armAngle.setPosition(.42);
                }

                if (gamepad2.dpad_down) {
                     //armAngle.setPosition(0);
                     runtime.reset();
                     servoProfile.generateProfile(.34, .23, .9, .3);
                     while (servoProfile.servoProfile1.get(runtime.seconds()).getX() < .29999 &&opModeIsActive() || runtime.seconds() < 3.5 && opModeIsActive()) {
                         bleftPower = Range.clip(drive - strafe - turn, -1, 1);
                         brightPower = Range.clip(drive + strafe + turn, -1, 1);
                         fleftPower = Range.clip(drive - strafe + turn, -1, 1);
                         frightPower = Range.clip(drive + strafe - turn, -1, 1);
                         intakePower = Range.clip(intake1, -.45, .45);
                         scoringleftPower = Range.clip(scoring, -0.65, 0.1);
                         scoringrightPower = Range.clip(scoring, -0.65, 0.1);
                         servoProfile.setServoPath();

                    }
                     armAngle.setPosition(.34);

//                    armAngle.setPosition(0);
//                    sleep(300);
//                    axonRight.setPosition(.5);
//                    axonLeft.setPosition(.4);
//                    sleep(100);
//                    axonRight.setPosition(.6);
//                    axonLeft.setPosition(.3);
//                    sleep(300);
//                    armAngle.setPosition(.295);
//                  axonRight.setPosition(6);
//                    axonLeft.setPosition(3);
//                    armAngle.setPosition(.34);
                }

                while (gamepad1.dpad_up) {
                    bleftDrive.setPower(0.15);
                    brightDrive.setPower(0.15);
                    fleftDrive.setPower(0.15);
                    frightDrive.setPower(0.15);
                }

                while (gamepad1.dpad_down) {
                    bleftDrive.setPower(-0.15);
                    brightDrive.setPower(-0.15);
                    fleftDrive.setPower(-0.15);
                    frightDrive.setPower(-0.15);
                }

                while (gamepad1.dpad_right) {
                    bleftDrive.setPower(-0.185);
                    brightDrive.setPower(0.185);
                    fleftDrive.setPower(0.185);
                    frightDrive.setPower(-0.185);
                }

                while (gamepad1.dpad_left) {
                    bleftDrive.setPower(0.185);
                    brightDrive.setPower(-0.185);
                    fleftDrive.setPower(-0.185);
                    frightDrive.setPower(0.185);
                }

//                while(gamepad1.right_bumper){
//                    hangLeft.setPower(0.25);
//                    hangRight.setPower(0.25);
//                }
//                while(gamepad2.left_bumper){
//
//                }

//                while (opModeIsActive() && hangLeft.getCurrentPosition() < hangLeft.getTargetPosition())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//                {
//                    telemetry.addData("encoder-fwd-left", hangLeft.getCurrentPosition() + "  busy=" + hangLeft.isBusy());
//                    telemetry.addData("encoder-fwd-right", hangRight.getCurrentPosition() + "  busy=" + hangRight.isBusy());
//                    telemetry.update();
//                    idle();
//                }

                if (gamepad2.left_trigger > 0.2) {
                    scoringservoRight.setPosition(0);
                }
                if (gamepad2.left_bumper) {
                    scoringservoRight.setPosition(.55);
                }

                if (gamepad2.right_trigger > 0.2) {
                    scoringservoLeft.setPosition(.55);
                }
                if (gamepad2.right_bumper) {
                    scoringservoLeft.setPosition(0);
                }




                // Send calculated power to wheels
                bleftDrive.setPower(bleftPower);
                brightDrive.setPower(brightPower);
                fleftDrive.setPower(fleftPower);
                frightDrive.setPower(frightPower);
                intake.setPower(intakePower);
                scoringRight.setPower(scoringrightPower);
                scoringLeft.setPower(scoringleftPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "bleftDrive (%100f), brightDrive (%100f)", bleftPower, brightPower);
                telemetry.addData("Set Position Left: ", hangLeft.getCurrentPosition());
                telemetry.addData("Set Position Right: ", hangRight.getCurrentPosition());
                telemetry.update();
            }
        }
    }


