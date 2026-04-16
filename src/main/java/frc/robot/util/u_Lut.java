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
        rpmMap.put(7.45, 1700.0);
        angleMap.put(7.45, 22.0);
        tofMap.put(7.45, .93);

        rpmMap.put(10.73, 1850.0);//short
        angleMap.put(10.73, 24.2);
        // tofMap.put(10.73, .79);

        rpmMap.put(13.39, 2000.0);//Short
        angleMap.put(13.39, 25.0);
        // tofMap.put(13.39, 1.33);

        rpmMap.put(15.42, 2100.0);
        angleMap.put(15.42, 25.0);
        // tofMap.put(15.42, 1.44);
        
        rpmMap.put(16.91, 2200.0);
        angleMap.put(16.91, 26.0);
        // tofMap.put(16.91, 1.38);

        
        rpmMap.put(18.22, 200.0);
        angleMap.put(18.22, 25.0);
        // tofMap.put(18.22, 1.44);

        tofMap.put(1000.22, 20.44);


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
