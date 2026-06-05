// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Emergency;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Zeroing extends Command {
  /** Creates a new Zeroing. */
  s_Turret s_Turret;
  double currentDeg = 0;
  DoubleSupplier rightStick;

  public Zeroing(s_Turret s_Turret, DoubleSupplier rightStick) {
    addRequirements(s_Turret);
    this.s_Turret = s_Turret;
    this.rightStick = rightStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDeg = 0;
    s_Turret.setDegrees(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDeg += rightStick.getAsDouble();
    SmartDashboard.putNumber("CurrentDegzeroing", currentDeg);
    s_Turret.setDegrees(currentDeg);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Turret.overridePosition(s_Turret.getAngle().in(Degrees));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
