// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.AutonomousCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PathPlanning;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SystemLog;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.FlightStick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public Command m_autonomousCommand;
  public SystemLog m_systemlog = SystemLog.getInstance();
  public DriveTrain m_drivetrain = DriveTrain.getInstance();
  public Gyro m_gyro = Gyro.getInstance();
  public PoseEstimator m_poseestimator = PoseEstimator.getInstance();
  public PathPlanning m_pathplanning = PathPlanning.getInstance();
  public Autonomous m_autonomous = Autonomous.getInstance();
  public Vision m_vision = Vision.getInstance();
  public LED m_led = LED.getInstance();
  // public Dashboard m_dash = Dashboard.getInstance();

  public IntakeShooter m_intake = IntakeShooter.getInstance();
  public Climber m_climberRight = Climber.right();
  public Climber m_climberLeft = Climber.left();

  Alliance myAlliance = Alliance.Red;

  public boolean changedAlly = true;

  // Trying to add Auto selection to the other dashboard

  public static Command doNothingCmd = Commands.sequence(
    new InstantCommand(()->DriveTrain.getInstance().m_poseQueue.clear()),
    new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up))
  );
  
  public static Command ScoreSpeakerCommands = Commands.sequence(
    AutonomousCommands.StorePose(),
    new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up)),
    new InstantCommand (()->DriveTrain.getInstance().m_poseQueue.clear()),
    Commands.waitSeconds(7), // Wait for wheels to start turning
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.ReturnToPose()
  );
  
  public static Command Score2SpeakerMobCommands = Commands.sequence(
    Commands.waitSeconds(3.5), // Wait for wheels to start turning
    AutonomousCommands.StorePose(),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,1),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,0),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.ReturnToPose()
  );

  public static Command Score2SpeakerCommands = Commands.sequence(
    AutonomousCommands.StorePose(),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.PickupNote(AutoConstants.kAllianceNotes,1),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.ReturnToPose()
  );

  public static Command Score3SpeakerCommands = Commands.sequence(
    AutonomousCommands.StorePose(),
    AutonomousCommands.ScoreSpeaker(),
    new InstantCommand(() -> PathPlanning.getInstance().navigateTo(PathPlanning.getInstance().nodes.get(0).getPose())),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,1),
    new InstantCommand(() -> PathPlanning.getInstance().navigateTo(PathPlanning.getInstance().nodes.get(0).getPose())),
    AutonomousCommands.ScoreSpeaker(),
    new InstantCommand(() -> PathPlanning.getInstance().navigateTo(PathPlanning.getInstance().nodes.get(0).getPose())),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,2),
    new InstantCommand(() -> PathPlanning.getInstance().navigateTo(PathPlanning.getInstance().nodes.get(0).getPose())),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.ReturnToPose()
  );

  public static Command ScoreAmpCommands = Commands.sequence(
    AutonomousCommands.StorePose(),
    AutonomousCommands.ScoreAmp(),
    AutonomousCommands.ReturnToPose()
  );

  public static Command Score2AmpMobCommands = Commands.sequence(
    AutonomousCommands.ScoreAmp(),
    AutonomousCommands.PickupNote(AutoConstants.kAllianceNotes,2),
    AutonomousCommands.ScoreAmp(),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,3),
    AutonomousCommands.ScoreSpeaker()
  );

  public static Command ScoreSpeakerMobCommands = Commands.sequence(
    AutonomousCommands.StorePose(),
    AutonomousCommands.ScoreSpeaker(),
    AutonomousCommands.PickupNote(AutoConstants.kCenterNotes,1)
  );

  // No Autonomous options below this
  
  String[] autoArray = {"Do Nothing"
  , "Score Speaker 7 second delay"
  , "Score 2 Note Speaker Mobility Notes"
  , "Score 2 Note Speaker Alliance Notes"
  , "Score 3 Note Speaker"
  , "Score Amp"
  , "Score 2 Amp Mobility"
  , "Score Speaker Mobility"
  , "Build Your Own A(uto)dventure"};

  {SmartDashboard.putStringArray("Auto List", autoArray);}

  @Override
  public void robotInit() {
    DataLogManager.start();
  }

  boolean noDS = true;
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    changedAlly = false;
    if ((noDS || myAlliance==Alliance.Blue) && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      myAlliance = Alliance.Red;
      changedAlly = true;
      noDS = false;
    } else if ((noDS || myAlliance==Alliance.Red) && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      myAlliance = Alliance.Blue;
      changedAlly = true;
      noDS = false;
    } else if (DriverStation.getAlliance().isPresent() == false){
      noDS = true;
    }

    if (changedAlly){
      FlightStick.m_blueAlly = (myAlliance == Alliance.Blue ? true : false);
      PoseEstimator.getInstance().reset();
      Gyro.getInstance().setGyroInit((myAlliance == Alliance.Blue ? 0 : Math.PI), 0, 0);
      AutoConstants.calcAllianceNotes(myAlliance == Alliance.Blue ? true : false);
      PathPlanning.getInstance().calcFieldGraph();
    }
  
  }

  @Override
  public void autonomousInit() {
    IntakeShooter.getInstance().m_aimForDistance = false;
    DriveTrain.getInstance().m_poseQueue.clear();

    switch(SmartDashboard.getString("Auto Selector", "Do Nothing")) {
      case "Do Nothing":
        m_autonomousCommand = doNothingCmd;
        break;
      case "Score Speaker 7 second delay":
        m_autonomousCommand = ScoreSpeakerCommands;
        break;
      case "Score 2 Note Speaker Mobility Notes":
        m_autonomousCommand = Score2SpeakerMobCommands;
        break;
      case "Score 2 Note Speaker Alliance Notes":
        m_autonomousCommand = Score2SpeakerCommands;
        break;
      case "Score 3 Note Speaker":
        m_autonomousCommand = Score3SpeakerCommands;
        break;
      case "Score Amp":
        m_autonomousCommand = ScoreAmpCommands;
        break;
      case "Score 2 Amp Mobility":
        m_autonomousCommand = Score2AmpMobCommands;
        break;
      case "Score Speaker Mobility":
        m_autonomousCommand = ScoreSpeakerMobCommands;
        break;
      case "Build Your Own A(uto)dventure":
        m_autonomousCommand = Dashboard.getInstance().BuildYourOwnAutoCommands();
        break;
   }
    CommandScheduler.getInstance().schedule(m_autonomousCommand);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    IntakeShooter.getInstance().m_aimForDistance = false;
    DriveTrain.getInstance().m_poseQueue.clear();
    Autonomous.getInstance().stopAiming();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    DriveTrain.getInstance().simulationInit();    
  }

  @Override
  public void simulationPeriodic() {
    //DriveTrain.getInstance().simulationPeriodic();
    //IntakeShooter.getInstance().simulationPeriodic();
    REVPhysicsSim.getInstance().run();
  }
}

//    AutonomousCommands.PickupNote(AutoConstants.kAllianceNotes,2),
//    AutonomousCommands.ScoreAmp(),
//    new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(AutoConstants.kCenterNotes.get(3))),
//    new InstantCommand(() -> PathPlanning.getInstance().navigateCloseTo(AutoConstants.kCenterNotes.get(3),Units.feetToMeters(3),Math.PI)
//    new InstantCommand (()->DriveTrain.getInstance().m_poseQueue.clear()),