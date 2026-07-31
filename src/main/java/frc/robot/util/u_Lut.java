// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class u_Lut {
    public u_Lut() {
    }

    private static boolean initialized = false;

    private static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private static void initialize() {
        tofMap.put(1.0, 1.0);

        rpmMap.put(5.416, 1900.0);
        angleMap.put(5.416, 23.0); 
        tofMap.put(5.416, .63);


        rpmMap.put(7.034, 2100.0);
        angleMap.put(7.034, 25.0); 
        tofMap.put(7.034, .94);


        rpmMap.put(9.060, 2200.0);
        angleMap.put(9.060, 26.0); 
        tofMap.put(9.060, 1.09);


        rpmMap.put(10.590, 2250.0);
        angleMap.put(10.590, 26.0); 
        tofMap.put(10.590, 1.08);


        rpmMap.put(11.335, 2300.0);
        angleMap.put(11.335, 27.0);
        tofMap.put(11.335, 1.07);


        rpmMap.put(13.607, 2475.0);
        angleMap.put(13.607, 27.0);
        tofMap.put(13.607, 1.13);

        rpmMap.put(16.009, 2560.0);
        angleMap.put(16.009, 31.0);
        tofMap.put(16.009, 1.16);

        rpmMap.put(18.470, 2600.0);
        angleMap.put(18.470, 34.0);
        tofMap.put(18.470, 1.11);

        rpmMap.put(21.053, 2750.0);
        angleMap.put(21.053, 34.0);
        tofMap.put(21.053, 1.45);

        rpmMap.put(23.062, 2890.0);
        angleMap.put(23.062, 35.0);
        tofMap.put(23.062, 1.35);

        rpmMap.put(25.400, 3030.0);
        angleMap.put(25.400, 36.0);
        tofMap.put(25.400, 1.52);


        initialized = true;
    }

    public static double getRpmFrom(double feet) {
        if (!initialized)
            initialize();

        return rpmMap.get(feet);
    }

    public static double getAngleFrom(double feet) {
        if (!initialized)
            initialize();

        return angleMap.get(feet);
    }

    public static double getTofFrom(double feet) {
        if (!initialized)
            initialize();

        return tofMap.get(feet);
    }

    static Pose2d currentGoalPosition;
    static Pose2d translatedGoalPose;
    static Pose2d translatedTurretPose;

    static ChassisSpeeds speeds;

    static double gears;

}
