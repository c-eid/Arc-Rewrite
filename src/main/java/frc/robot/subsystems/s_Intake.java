// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;


public class s_Intake extends SubsystemBase {
  /** Creates a new s_Intake. */
  public static s_Intake m_Instance;



  private SparkFlex intakeRollerBlack = new SparkFlex(32, MotorType.kBrushless);
  private SparkFlex intakeRollerBlue = new SparkFlex(33, MotorType.kBrushless);

  TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  Slot0Configs slot0Configs = pivotConfigs.Slot0;

  final MotionMagicExpoTorqueCurrentFOC m_request = new MotionMagicExpoTorqueCurrentFOC(0);

    private TalonFX lPivotTalonFX = new TalonFX(30);
  private TalonFX rPivotTalonFX = new TalonFX(31);

  public s_Intake() {
    initialized = true;

    SparkBaseConfig config = new SparkFlexConfig().openLoopRampRate(0.4).smartCurrentLimit(40);

    intakeRollerBlack.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    intakeRollerBlue.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kG = 0;

    slot0Configs.kP = 100;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = pivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicExpo_kV = 0;
    motionMagicConfigs.MotionMagicExpo_kA = 0;
    

    pivotConfigs.Feedback.SensorToMechanismRatio = 20;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    
    lPivotTalonFX.setControl(new Follower(31, MotorAlignmentValue.Opposed));

    rPivotTalonFX.getConfigurator().apply(pivotConfigs);
    lPivotTalonFX.getConfigurator().apply(pivotConfigs);
  }

  // private double currentDeg = 0.0;

  public static s_Intake getInstance(){
    if (m_Instance == null) {
      m_Instance = new s_Intake();
    }
    return m_Instance;
  }

  public void setDegrees(double deg){
    rPivotTalonFX.setControl(m_request.withPosition(Degrees.of(deg)));
  }

  // public void overrideDeg(double deg){
  // }

  public void setSpeed(double dutyCycle){
    intakeRollerBlack.set(dutyCycle);
    intakeRollerBlue.set(-dutyCycle);

  }

  public TalonFX getLeftPivotTalonFX(){
    return lPivotTalonFX;
  }

  public TalonFX getRightPivotTalonFx(){
    return rPivotTalonFX;
  }

  public boolean initialized = false; 

  public boolean checkSubsystem(){
    return getInitialized();
  }

  public boolean getInitialized(){
    return initialized;
  }

  public void stop(){
    intakeRollerBlack.stopMotor();
    intakeRollerBlue.stopMotor();

    lPivotTalonFX.stopMotor();
    rPivotTalonFX.stopMotor();
  }


  @Override
  public void periodic() {
  }

  
  @Override
  public void simulationPeriodic() {
  }
}
