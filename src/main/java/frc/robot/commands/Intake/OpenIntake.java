// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.s_Intake;

public class OpenIntake extends InstantCommand {
  s_Intake intake;

  public OpenIntake(s_Intake intake) {
      this.intake = intake;
      addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setDegrees(0);
    intake.setSpeed(0);
  }


}
