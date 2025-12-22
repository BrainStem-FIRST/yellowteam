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
        public LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;
        public String[] motorNames = new String[] { "turret", "lowShoot", "highShoot", "intake", "FL", "FR", "BL", "BR" };
        public int[] numGetPosition = new int[] { 10, 10, 10, 10, 10, 10, 10, 10 };
    }
    public static ThreadParams threadParams = new ThreadParams();
    public static HardwareParams hardwareParams = new HardwareParams();
    private Thread[] threads;
    private AtomicInteger workerCounter;
    private int mainCounter;
    private ElapsedTime timer;
    private AtomicBoolean keepRunning;
    private DcMotorEx[] motors;
    private List<LynxModule> allHubs;
    private int numTimesFunctionPerformedPerFrame;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(hardwareParams.bulkCachingMode);

        numTimesFunctionPerformedPerFrame = threadParams.numWorkerThreads + (threadParams.performFunctionOnMainThread ? 1 : 0);

        threads = new Thread[threadParams.numWorkerThreads];
        workerCounter = new AtomicInteger(0);
        mainCounter = 0;
        keepRunning = new AtomicBoolean(true);
        for (int i = 0; i< threadParams.numWorkerThreads; i++)
            threads[i] = new Thread(this::functionWorkerThreads);
        timer = new ElapsedTime();

        motors = new DcMotorEx[hardwareParams.motorNames.length];
        for (int i=0; i<hardwareParams.motorNames.length; i++)
            motors[i] = hardwareMap.get(DcMotorEx.class, hardwareParams.motorNames[i]);

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
        if (hardwareParams.bulkCachingMode == LynxModule.BulkCachingMode.MANUAL)
            for (LynxModule hub : allHubs)
                hub.clearBulkCache();

        if (threadParams.performFunctionOnMainThread) {
            functionOnce();
            mainCounter++;
        }

        telemetry.addData("perform function on main thread", threadParams.performFunctionOnMainThread);
        telemetry.addData("num worker threads", threadParams.numWorkerThreads);
        telemetry.addData("bulk caching mode", hardwareParams.bulkCachingMode);
        telemetry.addData("num trig functions", threadParams.numTrigFunctionsPerCounter);
        for (int i=0; i<motors.length; i++)
            telemetry.addData(motors[i] + " getPower calls", hardwareParams.numGetPosition[i]);
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
    private void functionOnce() {
        for (int i = 0; i < threadParams.numTrigFunctionsPerCounter / numTimesFunctionPerformedPerFrame; i++)
            Math.sin(0.5);
        for (int i = 0; i < motors.length; i++) {
            for (int j = 0; j < hardwareParams.numGetPosition[i] / numTimesFunctionPerformedPerFrame; j++)
                motors[i].getCurrentPosition();
        }
    }
    private void functionWorkerThreads() {
        while (keepRunning.get()) {
            functionOnce();
            workerCounter.getAndAdd(1);
        }
    }
}
