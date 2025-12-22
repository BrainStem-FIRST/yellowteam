package com.example.motionProfiling;

public class MotionProfileDisplay {
    public static void main(String[] args) {
        double totalError = 200;
        double stepSize = 5;

        TrapezoidalMotionProfiler profiler = new TrapezoidalMotionProfiler(totalError);

        for (double i=totalError; i>=0; i-=stepSize) {
            System.out.println(i + " | " + profiler.calculate(i));
        }
    }
}
