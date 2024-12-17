// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of the FRC 6328 RobotCode2024 project

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * 
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses.  When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock = 
        new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private boolean isCANFD = false;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        DriveSubsystem.odometryLock.lock();
        try {
            isCANFD = CANBus.isNetworkFD(device.getNetwork());
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            signalsLock.unlock();
            DriveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD) {
                    //TODO: Did not use variable here, 0.02 instead
                    BaseStatusSignal.waitForAll(0.02, signals);
                } else {
                    //TODO: This is set default to 250 - may need to change for SIM
                    //See 6328 thread
                    Thread.sleep((long) (1000.0 / 250));
                    if(signals.length > 0) BaseStatusSignal.refreshAll(signals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }
            double fpgaTimestamp = Logger.getRealTimestamp() / 1.0e6;

            // Save new data to queues
            DriveSubsystem.odometryLock.lock();
            try {
                for (int i = 0; i <signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
                DriveSubsystem.timestampQueue.offer(fpgaTimestamp);
            } finally {
                DriveSubsystem.odometryLock.unlock();
            }
        }
    }
    
}
