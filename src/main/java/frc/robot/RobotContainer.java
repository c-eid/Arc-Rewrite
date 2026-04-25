// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IO.TurretIO;
import frc.robot.commands.Aiming.HoodTrack;
import frc.robot.commands.Aiming.TurretTrack;
import frc.robot.commands.Intake.Intaking;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.drive.s_Drivetrain;
import frc.robot.subsystems.poseEstimation.s_QuestNav;
import frc.robot.util.Touchboard;
import frc.robot.util.u_Dist;

public class RobotContainer {
  
  //Controllers 
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController copilot = new CommandXboxController(1);

  //IO
  TurretIO turretSimulation = new TurretIO();

  //Subsystems
  s_Drivetrain s_Swerve = new s_Drivetrain();
  s_QuestNav s_QNav = new s_QuestNav(s_Swerve.getDrivetrain());
  s_Intake s_Intake = new s_Intake();
  s_Turret s_Turret = new s_Turret(turretSimulation);
  s_Hood s_Hood = new s_Hood(turretSimulation);

  //Util 
  u_Dist u_Dist = new u_Dist(s_Swerve);

  //Commands
  Intaking intake_Com = new Intaking(s_Intake);
  HoodTrack hoodTrack = new HoodTrack(s_Hood, u_Dist);
  TurretTrack turretTrack = new TurretTrack(s_Turret, u_Dist);
  
  
  public RobotContainer() {
    s_Swerve.bindControllers(s_QNav, driver);

    configureDefaults();  
    configureModifierBindings();
    configureCommandBindings();
  }


  private void configureDefaults(){
    s_Turret.setDefaultCommand(turretTrack);
    s_Hood.setDefaultCommand(hoodTrack);
  }

  private void configureModifierBindings(){

  }

  private void configureCommandBindings(){
    driver.rightBumper().whileTrue(intake_Com);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
