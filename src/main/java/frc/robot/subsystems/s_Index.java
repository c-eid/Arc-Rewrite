// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class s_Index extends SubsystemBase {
  /** Creates a new s_Shooter. */

  private TalonFX indexTalon = new TalonFX(50);
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  public s_Index() {
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    indexTalon.getConfigurator().apply(talonFXConfigs);
  }

  boolean jamming = false;
  double indexVelocity = 0;
  double startJamTimestamp = 0;

  public void setIndexRpm(double rpm) {
    indexTalon.setControl(m_request.withVelocity(RPM.of(rpm)));
  }

  public double getVelocity() {
    return indexTalon.getVelocity().getValue().in(RPM);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
