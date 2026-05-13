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
        // rpmMap.put(4.782, 1850.0);
        // angleMap.put(4.782, 20.0);
        // tofMap.put(4.782, .75);

        
        //demo 
        rpmMap.put(3.5, 1100.0);
        angleMap.put(3.5, 30.0);
        tofMap.put(3.5, .20);

         rpmMap.put(6.07, 1400.0);
        angleMap.put(6.07, 37.0);
        tofMap.put(6.07, .34);

        rpmMap.put(7.06, 1550.0);
        angleMap.put(7.06, 40.0);
        tofMap.put(7.06, .40);

        rpmMap.put(9.03, 1850.0);
        angleMap.put(9.03, 40.0);
        tofMap.put(9.03, .60);


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
