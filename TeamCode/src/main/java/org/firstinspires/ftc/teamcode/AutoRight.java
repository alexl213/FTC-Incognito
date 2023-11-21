package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="AutoRight", group="Linear Opmode")

public class AutoRight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lily ; //left drive
    private DcMotor Ronie ; //right drive
    private DcMotor Klaus ; //chain and sprocket linear motor
    private DcMotor Gladys ; //launch path
    private DcMotor Timothy ;
    private DcMotor Felicity ; //flip motor
    private Servo Sergio ;//flip down servo

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Lily = hardwareMap.get(DcMotor.class, "Lily");
        Ronie = hardwareMap.get(DcMotor.class, "Ronie");
        Klaus = hardwareMap.get(DcMotor.class, "Klaus");
        Gladys = hardwareMap.get(DcMotor.class, "Gladys");
        Timothy = hardwareMap.get(DcMotor.class, "Timothy");
        Felicity = hardwareMap.get(DcMotor.class, "Felicity");
        Sergio = hardwareMap.get(Servo.class,"Sergio");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //drop intake
        Sergio.setPosition(0);

        sleep(2000);//wait

        Lily.setPower(1);//go to launch line
        Ronie.setPower(-1);
        sleep(1700);//originally 1500
        Lily.setPower(0);
        Ronie.setPower(0);

        sleep(1000);

        Klaus.setPower(0.5);//move ramp down to score top goal
        sleep(1290);// on 13.07 V 3000 for low goal //1500 for middle goal //1100 for top goal
        Klaus.setPower(0);

        sleep(1000);

        Lily.setPower(-1);//turn to right
        Ronie.setPower(-1);
        sleep(150);//80 degrees at 500
        Lily.setPower(0);
        Ronie.setPower(0);

        Gladys.setPower(1);//start flywheel
        sleep(1500);
        Timothy.setPower(-1);//start track
        sleep(4000);//adjust to fit delivery needs//launch rings into top goal
        Gladys.setPower(0);
        Timothy.setPower(0);
        sleep(1000);

        Lily.setPower(0.5);//park on launch line
        Ronie.setPower(-0.5);
        sleep(1000);
        Lily.setPower(0);
        Ronie.setPower(0);


    }
}

