// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Intake;

public class HomeIntake extends Command {
  s_Intake intake;
  boolean leftHomed = false;
  boolean rightHomed = false;
  Supplier<TalonFX> right;
  Supplier<TalonFX> left;

  public HomeIntake(s_Intake intake) {
      this.intake = intake;
      addRequirements(intake);

      right = () -> intake.getRightPivotTalonFx();
      left = () -> intake.getLeftPivotTalonFX();

    }

  @Override
  public void initialize() {
    leftHomed = false;
    rightHomed = false; 

    intake.setLeftVoltage(-2);
    intake.setRightVoltage(-2);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(right.get().getStatorCurrent().getValue().gte(Amps.of(20))){
      rightHomed = true;
    }

    if(left.get().getStatorCurrent().getValue().gte(Amps.of(20))){
      leftHomed = true;
    }

    System.out.println(rightHomed + " , " + leftHomed);
    System.out.println(right.get().getStatorCurrent().getValue().in(Amps) + ", " + left.get().getStatorCurrent().getValue().in(Amps));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
    intake.overrideDeg(3);

    left.get().setControl(new Follower(31, MotorAlignmentValue.Aligned));

    intake.setDegrees(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftHomed && rightHomed;
  }
}
