package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DrivingLogic;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoRedCloseSpikeReturn")

public class AutoRedCloseSpikeReturn extends LinearOpMode {

    private Servo scoringservoLeft = null;
    private Servo scoringservoRight = null;
    double cX = 0;
    double cY = 0;
    double width = 0;
    public ServoProfile servoProfile = new ServoProfile();
    public DrivingLogic robot = new DrivingLogic(hardwareMap, gamepad1);
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
    private DcMotor hangLeft = null;
    private DcMotor hangRight = null;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1531.42;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {

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
        robot.driveMotorInit(frightDrive, fleftDrive,bleftDrive,brightDrive);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        initOpenCV();

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            double intakePower;
            double scoringleftPower;
            double scoringrightPower;
            double intake1 = gamepad2.left_stick_x;
            double scoring = gamepad2.right_stick_y;
            intakePower = Range.clip(intake1, -.45, .45);
            scoringleftPower = Range.clip(scoring, -0.65, 0.1);
            scoringrightPower = Range.clip(scoring, -0.65, 0.1);


            Pose2d startPose = new Pose2d(12, -63, Math.toRadians(-90));
            Pose2d rightPose = new Pose2d(12, -37, Math.toRadians(0.0));
            Pose2d leftPose = new Pose2d(12, -31, Math.toRadians(180.0));
            Pose2d forwardPose = new Pose2d(12, -37, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            Trajectory closeright1 = drive.trajectoryBuilder(startPose)
                    .forward(-26.0)
                    .build();
            Trajectory closeright2 = drive.trajectoryBuilder(rightPose)
                    .forward(3.5)
                    .build();
            Trajectory closeright3 = drive.trajectoryBuilder(closeright2.end())
                    .forward(-3.5)
                    .build();
            Trajectory closeright4 = drive.trajectoryBuilder(closeright3.end())
                    .strafeRight(26.0)
                    .build();
            Trajectory closeleft1 = drive.trajectoryBuilder(startPose)
                    .forward(-32.0)
                    .build();
            Trajectory closeleft2 = drive.trajectoryBuilder(leftPose)
                    .forward(3.5)
                    .build();
            Trajectory closeleft3 = drive.trajectoryBuilder(closeleft2.end())
                    .forward(-3.5)
                    .build();
            Trajectory closeleft4 = drive.trajectoryBuilder(closeleft3.end())
                    .strafeLeft(28)
                    .build();
            Trajectory closeforward1 = drive.trajectoryBuilder(startPose)
                    .forward(-26.0)
                    .build();
            Trajectory closeforward2 = drive.trajectoryBuilder(forwardPose)
                    .forward(3.5)
                    .build();
            Trajectory closeforward3 = drive.trajectoryBuilder(closeforward2.end())
                    .forward(-29.5)
                    .build();

            if (opModeIsActive() && cX > 1350) {
                telemetry.addData("Location: ", "Right");
                telemetry.update();
                servoProfile.initServos(axonLeft, axonRight);
                scoringservoLeft.setPosition(.1);
                scoringservoRight.setPosition(.3);
                Servo armAngle = null;
                armAngle = hardwareMap.get(Servo.class, "armAngle");
                armAngle.setPosition(.36);
                sleep(500);
                controlHubCam.stopStreaming();
                drive.followTrajectory(closeright1);
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(closeright2);
                armAngle.setPosition(.0);
                sleep(300);
                scoringservoLeft.setPosition(.32);
                drive.followTrajectory(closeright3);
                armAngle.setPosition(.36);
                scoringservoLeft.setPosition(.05);
                scoringservoRight.setPosition(.32);
                sleep(100);
                drive.followTrajectory(closeright4);
                sleep(100000000);
            }
            if (opModeIsActive() && cX < 550) {
                telemetry.addData("Location: ", "Left");
                telemetry.update();
                servoProfile.initServos(axonLeft, axonRight);
                scoringservoLeft.setPosition(.1);
                scoringservoRight.setPosition(.3);
                Servo armAngle = null;
                armAngle = hardwareMap.get(Servo.class, "armAngle");
                armAngle.setPosition(.36);
                sleep(500);
                controlHubCam.stopStreaming();
                drive.followTrajectory(closeleft1);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(closeleft2);
                armAngle.setPosition(.0);
                sleep(300);
                scoringservoLeft.setPosition(.32);
                drive.followTrajectory(closeleft3);
                armAngle.setPosition(.36);
                scoringservoLeft.setPosition(.05);
                scoringservoRight.setPosition(.32);
                sleep(100);
                drive.followTrajectory(closeleft4);
                sleep(100000000);
            }
            if (opModeIsActive() && cX < 1350 && cX > 550) {
                telemetry.addData("Location: ", "Center");
                telemetry.update();
                servoProfile.initServos(axonLeft, axonRight);
                scoringservoLeft.setPosition(.1);
                scoringservoRight.setPosition(.3);
                Servo armAngle = null;
                armAngle = hardwareMap.get(Servo.class, "armAngle");
                armAngle.setPosition(.36);
                sleep(500);
                controlHubCam.stopStreaming();
                drive.followTrajectory(closeforward1);
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(closeforward2);
                armAngle.setPosition(.0);
                sleep(300);
                scoringservoLeft.setPosition(.32);
                sleep(300);
                armAngle.setPosition(.36);
                scoringservoLeft.setPosition(.05);
                scoringservoRight.setPosition(.32);
                sleep(100);
                drive.followTrajectory(closeforward3);
                sleep(100000000);

            }

        }
        controlHubCam.stopStreaming();
    }


    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class YellowBlobDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 50, 50);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }

        private static double getDistance(double width) {
            double distance = (objectWidthInRealWorldUnits * focalLength) / width;
            return distance;
        }


    }

