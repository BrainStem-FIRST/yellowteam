package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.concurrent.atomic.AtomicInteger;

@Config
@TeleOp(name="Multithreading Test", group="Testing")
public class MultiThreadingTest extends OpMode {
    public static int numThreads = 1;
    public static double numTrigFunctionsPerCounter = 50;
    private Thread[] threads;
    private AtomicInteger counter;
    private ElapsedTime timer;
    @Override
    public void init() {
        threads = new Thread[numThreads];
        counter = new AtomicInteger(0);
        for (int i=0; i<numThreads; i++)
            threads[i] = new Thread(this::function);
        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        for (Thread thread : threads)
            thread.start();
        timer.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("counter", counter);
        telemetry.addData("time", MathUtils.format3(timer.seconds()));
        telemetry.addData("avg count/sec", MathUtils.format3(counter.get() / timer.seconds()));
        telemetry.update();
    }
    private void function() {
        for (int i=0; i<numTrigFunctionsPerCounter; i++)
            Math.sin(0.5);
        counter.set(counter.get() + 1);
    }
}
