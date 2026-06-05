package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class MotorJamDetector {
    // Tunable parameters based on your mechanism's physical traits
    private final double currentThresholdAmps;
    private final double velocityThreshold;
    private final double allowedSpinUpTimeSeconds;

    private final Timer jamTimer = new Timer();
    private boolean isTimerRunning = false;

    /**
     * @param currentThresholdAmps High current limit indicating the motor is straining/stalled.
     * @param velocityThreshold Velocity below which the motor is considered "stuck".
     * @param allowedSpinUpTimeSeconds Time allowed for the motor to freely accelerate before checking for jams.
     */
    public MotorJamDetector(double currentThresholdAmps, double velocityThreshold, double allowedSpinUpTimeSeconds) {
        this.currentThresholdAmps = currentThresholdAmps;
        this.velocityThreshold = velocityThreshold;
        this.allowedSpinUpTimeSeconds = allowedSpinUpTimeSeconds;
        
        jamTimer.reset();
    }

    /**
     * Updates the jam detection logic. Call this periodically (e.g., in a subsystem's periodic() method).
     * * @param currentAmps Current stator/supply current from the motor controller.
     * @param currentVelocity Current velocity from the motor encoder.
     * @return true if a jam is detected and sustained past the spin-up window.
     */
    public boolean update(double currentAmps, double currentVelocity) {
        // Condition: Motor is drawing heavy current but barely moving
        boolean looksLikeStall = (Math.abs(currentAmps) > currentThresholdAmps) && 
                                 (Math.abs(currentVelocity) < velocityThreshold);

        if (looksLikeStall) {
            if (!isTimerRunning) {
                // Start timing how long this stall condition has existed
                jamTimer.reset();
                jamTimer.start();
                isTimerRunning = true;
            }

            // If the stall condition persists longer than the allowed spin-up/acceleration window
            if (jamTimer.get() >= allowedSpinUpTimeSeconds) {
                return true; // Confirmed jam
            }
        } else {
            // Conditions are normal; reset the window
            if (isTimerRunning) {
                jamTimer.stop();
                jamTimer.reset();
                isTimerRunning = false;
            }
        }

        return false; // No jam detected yet
    }
    
    /**
     * Resets the internal timer tracking. Call this when explicitly switching 
     * motor states or reversing intentionally.
     */
    public void reset() {
        jamTimer.stop();
        jamTimer.reset();
        isTimerRunning = false;
    }
}