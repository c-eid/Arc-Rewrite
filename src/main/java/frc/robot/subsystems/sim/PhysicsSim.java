package frc.robot.subsystems.sim;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }

    /**
     * Adds a TalonFX controller to the simulator.
     * 
     * @param talonFX
     *                  The TalonFX device
     * @param moi
     *                  rotational Inertia of the mechanism
     * @param gearRatio
     *                  ratio from rotor to mechanism
     */
    public void addTalonFX(TalonFX talonFX, final double moi, double gearRatio) {
        double rotorInertia = (moi / Math.pow(gearRatio, 2)) + 0.0000487; // 0.0000487 is the estimated inertia of the
                                                                          // motor itself, this is added to make sim
                                                                          // more accurate

        if (talonFX != null) {
            TalonFXSimProfile simTalonFX = new TalonFXSimProfile(talonFX, rotorInertia);
            _simProfiles.add(simTalonFX);
        }
    }

    public void addTalonFX(TalonFX talonFX, final double moi, double gearRatio, int gearBoxes) {
        double rotorInertia = (moi / Math.pow(gearRatio, 2)) + 0.0000487; // 0.0000487 is the estimated inertia of the
                                                                          // motor itself, this is added to make sim
                                                                          // more accurate

        if (talonFX != null) {
            TalonFXSimProfile simTalonFX = new TalonFXSimProfile(talonFX, rotorInertia, gearBoxes);
            _simProfiles.add(simTalonFX);
        }
    }

    /**
     * Adds a TalonFXS controller to the simulator.
     * 
     * @param talonFXS
     *                  The TalonFXS device
     * @param moi
     *                  rotational Inertia of the mechanism
     * @param gearRatio
     *                  ratio from rotor to mechanism
     */
    public void addTalonFXS(TalonFXS talonFXS, final double moi, double gearRatio) {
        double rotorInertia = (moi / Math.pow(gearRatio, 2)) + 0.0000487; // 0.0000487 is the estimated inertia of the
                                                                          // motor itself, this is added to make sim
                                                                          // more accurate

        if (talonFXS != null) {
            TalonFXSSimProfile simTalonFX = new TalonFXSSimProfile(talonFXS, rotorInertia);
            _simProfiles.add(simTalonFX);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private double _lastTime;
        private boolean _running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */
        public void run() {
        }

        /**
         * Returns the time since last call, in seconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = Utils.getCurrentTimeSeconds();
                _running = true;
            }

            double now = Utils.getCurrentTimeSeconds();
            final double period = now - _lastTime;
            _lastTime = now;

            return period;
        }
    }
}