package org.firstinspires.ftc.teamcode.TELEOP;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    // calibration for Logitech C270 @ 640x480 (Standard FTC Example Values)
    // If you use a different camera, you should calibrate it using the SDK Utility.
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // Changed to Inch/Deg for readability
                .setLensIntrinsics(fx, fy, cx, cy) // <--- FIX: Stops "No Calibration" error
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true); // Explicitly enable view
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // <--- FIX: Helps Stream show on Driver Hub
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        // The processor updates automatically in the background,
        // but we pull the list here for your TeleOp to use.
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public List<Double> displayDetectionTelemetry(AprilTagDetection detectedId) {

        if (detectedId == null) {
            telemetry.addLine("No Tag Detected");
            telemetry.update();
            // Return null if no tag is detected, which is correctly handled in your TeleOp
            return null;
        }
        // Clear the telemetry display before adding new data for a clean update
        telemetry.clear();

        List<Double> TagCoords = new ArrayList<>();

        if (detectedId == null) {
            telemetry.addLine("No Tag Detected");
            // Always call update, even if no tag is detected, to show the "No Tag Detected" message
            telemetry.update();
        } else {
            telemetry.addLine(String.format("Tag ID : " + detectedId.id));

            // NOW for the real values to be USED in the program
            TagCoords.add(detectedId.ftcPose.x);
            TagCoords.add(detectedId.ftcPose.y);
            TagCoords.add(detectedId.ftcPose.yaw);

            telemetry.addLine(String.format("X " + TagCoords.get(0)));
            telemetry.addLine(String.format("Y " + TagCoords.get(1)));
            telemetry.addLine(String.format("Yaw " + TagCoords.get(2)));

            telemetry.update();
        }

        return TagCoords;
    }
    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null){
            visionPortal.close();
        }
    }
}