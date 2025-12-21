package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@TeleOp(name="Multithreading Test", group="Testing")
public class MultiThreadingTest extends OpMode {
    public static class ThreadParams {
        public boolean performFunctionOnMainThread = true;
        public int numWorkerThreads = 0;
        public double numTrigFunctionsPerCounter = 0;
    }
    public static class HardwareParams {
        public boolean useBulkCaching = false;
        public String motorName = "turret";
        public int numGetPowers = 10;
    }
    public static ThreadParams threadParams = new ThreadParams();
    public static HardwareParams hardwareParams = new HardwareParams();
    private Thread[] threads;
    private AtomicInteger workerCounter;
    private int mainCounter;
    private ElapsedTime timer;
    private AtomicBoolean keepRunning;
    private DcMotorEx motor;
    private List<LynxModule> allHubs;
    private int numTimesFunctionPerformedPerFrame;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        if (hardwareParams.useBulkCaching) {
            allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        numTimesFunctionPerformedPerFrame = threadParams.numWorkerThreads + (threadParams.performFunctionOnMainThread ? 1 : 0);

        threads = new Thread[threadParams.numWorkerThreads];
        workerCounter = new AtomicInteger(0);
        mainCounter = 0;
        keepRunning = new AtomicBoolean(true);
        for (int i = 0; i< threadParams.numWorkerThreads; i++)
            threads[i] = new Thread(this::function);
        timer = new ElapsedTime();

        motor = hardwareMap.get(DcMotorEx.class, hardwareParams.motorName);

        telemetry.addLine("ready");
        telemetry.update();
    }

    @Override
    public void start() {
        for (Thread thread : threads)
            thread.start();
        timer.reset();
    }

    @Override
    public void loop() {
        if (hardwareParams.useBulkCaching)
            for (LynxModule hub : allHubs)
                hub.clearBulkCache();

        if (threadParams.performFunctionOnMainThread) {
            for (int i = 0; i<threadParams.numTrigFunctionsPerCounter / numTimesFunctionPerformedPerFrame; i++)
                Math.sin(0.5);
            for (int i = 0; i<hardwareParams.numGetPowers / numTimesFunctionPerformedPerFrame; i++)
                motor.getPower();
            mainCounter++;
        }

        telemetry.addData("perform function on main thread", threadParams.performFunctionOnMainThread);
        telemetry.addData("num worker threads", threadParams.numWorkerThreads);
        telemetry.addData("using bulk caching", hardwareParams.useBulkCaching);
        telemetry.addData("num trig functions", threadParams.numTrigFunctionsPerCounter);
        telemetry.addData("num getPower calls", hardwareParams.numGetPowers);
        telemetry.addLine();
        telemetry.addData("main counter", mainCounter);
        telemetry.addData("worker counter", workerCounter.get());
        telemetry.addData("time", MathUtils.format3(timer.seconds()));
        telemetry.addData("avg total count/sec", MathUtils.format3((workerCounter.get() + mainCounter) / timer.seconds()));
        telemetry.update();
    }

    @Override
    public void stop() {
        keepRunning.set(false);
    }
    private void function() {
        while (keepRunning.get()) {
            for (int i = 0; i<threadParams.numTrigFunctionsPerCounter / numTimesFunctionPerformedPerFrame; i++)
                Math.sin(0.5);
            for (int i = 0; i<hardwareParams.numGetPowers / numTimesFunctionPerformedPerFrame; i++)
                motor.getPower();

            workerCounter.getAndAdd(1);
        }
    }
}
