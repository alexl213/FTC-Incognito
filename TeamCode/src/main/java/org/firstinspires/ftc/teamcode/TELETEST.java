package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DrivingLogic;

@TeleOp(name="TESTTELE", group="Linear Opmode")
public class TELETEST extends LinearOpMode {


        public static double Kg = -0.125; //-0.12
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
        private Servo planeShoot = null;
        private ServoProfile servoProfile = new ServoProfile();
        private DrivingLogic robot = new DrivingLogic(hardwareMap, gamepad1);
        private FtcDashboard dashboard = FtcDashboard.getInstance();
        private Telemetry dashboardTelemetry = dashboard.getTelemetry();
        private RevBlinkinLedDriver clawLED;
        private MecanumDrive mecanumDrive;
        IMU imu;//dont remember if this is correct IMU name
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        planeShoot = hardwareMap.get(Servo.class, "plane_Shoot");
        robot.driveMotorInit(frightDrive, fleftDrive,bleftDrive,brightDrive);
        imu = hardwareMap.get(IMU.class, "imu");
        robot.fieldCentricDriveInit(imu);
        clawLED = hardwareMap.get(RevBlinkinLedDriver.class, "clawLED");





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

        scoringLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoringRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
            robot.fieldCentricOnly(imu, gamepad1);
        }

        }

    }
