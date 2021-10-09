package org.firstinspires.ftc.teamcode.util;

public class Timer {

    private double startTime;
    private double accumulatedTime;
    private boolean running;

    private Object lock = new Object();

    public Timer() {
        reset();
    }

    public void start() {
        synchronized (lock) {
            if (!running) {
                startTime = System.currentTimeMillis() / 1000.0;
                running = true;
            }
        }
    }

    public void stop() {
        synchronized (lock) {
            running = false;
            accumulatedTime = get();
        }
    }

    public double get() {
        synchronized (lock) {
            if (running) {
                return (System.currentTimeMillis() / 1000.0) - startTime;
            } else {
                return accumulatedTime;
            }
        }
    }

    public void reset() {
        synchronized (lock) {
            accumulatedTime = 0;
            startTime = System.currentTimeMillis() / 1000.0;
        }
    }

    public boolean hasElapsed(double seconds) {
        synchronized (lock) {
            return get() > seconds;
        }
    }
}
