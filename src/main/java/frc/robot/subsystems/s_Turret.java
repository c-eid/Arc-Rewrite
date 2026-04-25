// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.isSim;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.IO.TurretIO;
import frc.robot.subsystems.sim.PhysicsSim;

public class s_Turret extends SubsystemBase {

  // Cosntants
  private final int motorId = 3;
  private final double ratio = (144 / 15) * 5 * 1.084;
  private final double moi = 0.106165886;

  // MotionMagicExpoSetup
  private final TalonFX turretMotor = new TalonFX(motorId);

  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  private Slot0Configs pivotSlot0Configs = pivotConfigs.Slot0;
  private MotionMagicConfigs pivotMotionMagicConfigs = pivotConfigs.MotionMagic;

  final MotionMagicExpoTorqueCurrentFOC m_request = new MotionMagicExpoTorqueCurrentFOC(0);

  // Turret Specific Variables
  public Supplier<Boolean> inaccurate = () -> false;
  private DoubleSupplier offset = () -> 0;

  // Telemetry
  private TurretIO turretSimulation;

  public s_Turret(TurretIO turretSim) {
    this.turretSimulation = turretSim;

    pivotSlot0Configs.kS = 0;
    pivotSlot0Configs.kV = 0; // Unnessasary
    pivotSlot0Configs.kA = 2;
    pivotSlot0Configs.kG = 0;

    pivotSlot0Configs.kP = 0; // 10;
    pivotSlot0Configs.kI = 0;
    pivotSlot0Configs.kD = 0;// 0.6;

    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = 2;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 4;

    pivotMotionMagicConfigs.MotionMagicExpo_kV = 0.124 * ratio;
    pivotMotionMagicConfigs.MotionMagicExpo_kA = 0.1 * ratio;

    pivotConfigs.Feedback.SensorToMechanismRatio = ratio;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 70;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 60;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.convertFrom(360, Degree);
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotations.convertFrom(-360, Degree);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    double rotorInertia = (moi / Math.pow(ratio, 2)) + 0.0000487; // 0.0000487 is the estimated inertia of the motor
                                                                  // itself, this is added to make sim more accurate

    PhysicsSim.getInstance().addTalonFX(turretMotor, rotorInertia);

    turretMotor.getConfigurator().apply(pivotConfigs);

    System.out.println("Rotor Inertia: " + rotorInertia);

  }

  public void setDegrees(double degrees) {
    // Add Offset%
    degrees += offset.getAsDouble();

    // Actual Motor position
    double motorPosition = turretMotor.getPosition().getValue().in(Degree);

    // Check innacuracy by seeing if a 10 degree tolerance
    if (Math.abs(degrees - motorPosition) <= 10) {
      inaccurate = () -> false;
    } else {
      inaccurate = () -> true;
    }

    SmartDashboard.putNumber("Turret/Setpoint", degrees);

    // There was a check to see if degree

    // Check existed for sim to show actual degrees but I might just make a seperate
    // telemetry method
    // Showed velocity as well might be useful

    turretMotor.setControl(m_request.withPosition(Rotations.convertFrom(degrees, Degree)));
  }

  public void setOffset(double offset) {
    // Telemetry for offset

    this.offset = () -> offset;
  }

  public void overridePosition(double degrees) {
    turretMotor.setPosition(Degree.of(degrees));
  }

  public Angle getAngle() {
    return turretMotor.getPosition().getValue();
  }

  @Override
  public void periodic() {
    turretSimulation.setTurretDegrees(this.getAngle().in(Degree));

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Turret/Angle", this.getAngle().in(Degree));

    SmartDashboard.putNumber("Turret/Velocity", turretMotor.getVelocity().getValue().in(RotationsPerSecond));
  }
}
