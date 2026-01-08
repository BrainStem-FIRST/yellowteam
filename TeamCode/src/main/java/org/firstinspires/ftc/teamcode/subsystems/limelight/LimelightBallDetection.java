package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;

import java.util.Arrays;

public class LimelightBallDetection extends LLParent {
    public static class Params {
        public double kp = 0.05;
        public double minStrafePower = 0.2;
        public double txErrorThreshold = 5;
        public int maxConsecutiveFails = 3;
    }

    public static Params params = new Params();
    private double[] pythonOutputs;
    private boolean successful;
    private int numTimesFailed;
    private double strafePower;
    private double tx;
    private int pipelineIndex;
    public LimelightBallDetection(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        pythonOutputs = new double[3];
        successful = false;
        strafePower = 0;
        tx = 0;
        numTimesFailed = 0;
    }

    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();
        double[] inputs = new double[1];
        limelight.updatePythonInputs(inputs);
        pythonOutputs = result.getPythonOutput();
        pipelineIndex = result.getPipelineIndex();

        successful = pythonOutputs[0] > 0;

        if (successful) {
            numTimesFailed = 0;
            tx = pythonOutputs[1];
            strafePower = -tx * params.kp;
            strafePower = Math.signum(strafePower) * Math.max(Math.abs(strafePower), params.minStrafePower);
        }
        else {
            numTimesFailed++;

            if (numTimesFailed > params.maxConsecutiveFails) {
                tx = 0;
                strafePower = 0;
            }
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("python outputs", Arrays.toString(pythonOutputs));
        telemetry.addData("pipeline index", pipelineIndex);
    }

    public boolean isSuccessful() {
        return successful;
    }
    public double getStrafePower() {
        return strafePower;
    }
    public double getTargetX() {
        return tx;
    }
}

/*
import cv2
import numpy as np
import math

def runPipeline(image, llrobot):
    # finding color contours===============================
    image = cv2.GaussianBlur(image, (3, 3), 0)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    successful, mask, x, y = get_max_area_pos(img_hsv,
        (60, 90, 130), (105, 255, 255),
        (140, 0, 130), (155, 255, 255))

    # contour segmentation================================

    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    #show_sample_pixel(image, img_hsv, 300, 200)
    successful_num = 1 if successful else -1
    return contour_at_point(x, y), image, [ successful_num, x, y ]

def get_max_area_pos(hsv, lower_bounds, upper_bounds, lower_bounds2, upper_bounds2):
    mask1 = cv2.inRange(hsv, lower_bounds, upper_bounds)
    mask2 = cv2.inRange(hsv, lower_bounds2, upper_bounds2)
    mask = cv2.bitwise_or(mask1, mask2)
    # find contours in the new binary image
    contours, _ = cv2.findContours(mask,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    shrink_size = (8, 8)
    grow_size = (30, 30)
    shrink = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, shrink_size)
    grow = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, grow_size)
    mask = cv2.erode(mask, shrink)
    mask = cv2.dilate(mask, grow)

    min_area = 1000

    max_area = 0
    x = 0
    y = 0
    for c in contours:
        area = cv2.contourArea(c)
        if (area > max_area):
            max_area = area
            x, y = centroid_xy(c)

    successful = max_area > min_area
    print(f"{successful} {max_area}")

    return successful, mask, x, y

# drawing functions========================
def show_sample_pixel(image, img_hsv, samplex, sampley):
    h, s, b = img_hsv[sampley, samplex]
    cv2.rectangle(image, (0, 60), (360, 120), (0, 0, 0), -1)
    cv2.putText(
        image, f"color: {h} {s} {b}", (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0, (0, 255, 0), 2,
        cv2.LINE_AA
    )
# utility functions=========================
def centroid_x(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m10"] / M["m00"]

def centroid_y(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m01"] / M["m00"]

def centroid_xy(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0, 0.0
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy

def contour_at_point(x, y, size=6):
    # contour shape: (N, 1, 2) int32
    s = size
    pts = np.array([
        [x - s, y - s],
        [x + s, y - s],
        [x + s, y + s],
        [x - s, y + s],
    ], dtype=np.int32).reshape((-1, 1, 2))
    return pts

 */