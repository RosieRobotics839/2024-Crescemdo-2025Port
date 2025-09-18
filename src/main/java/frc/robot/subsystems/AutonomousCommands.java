package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.VectorUtils;

public class AutonomousCommands {

    // Ignore this weird hack. If you don't have a constant from the class used (outside of a lambda, I guess) it doesn't create the NTDoubles.
    public static double dummyvalue = AutoConstants.kSpeakerDistance; // lol

    static public Command noop(){return new InstantCommand(()->{});};

    // Helper functions for april tag selection on blue vs red alliance.
    static public int speakerTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4 ); }
    static public int ampTag(){     return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5 ); }
    static public int sourceLTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 2 : 9 ); }
    static public int sourceRTag(){ return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 1 : 10); }
   
    public static Command ScoreSpeaker() {
        return Commands.sequence(
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up)),
            new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(PathPlanning.AprilTagAtDistance(speakerTag(),ShooterConstants.kSpeakerAimDistance),Units.degreesToRadians(180))), // continuously aim at speaker
            new InstantCommand(() -> PathPlanning.getInstance().navigateCloseTo(PathPlanning.AprilTagAtDistance(speakerTag(),0), AutoConstants.kSpeakerDistance, AutoConstants.kSpeakerMaxAngle, true)), 
            // Commands.waitSeconds(0.5), // Wait for wheels to start turning
            Commands.waitUntil(() -> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,DriveTrain.getInstance().m_poseQueue.peekLast(),AutoConstants.kSpeakerNearDistance,Math.PI)).unless(()->DriveTrain.getInstance().m_poseQueue.isEmpty()).withTimeout(8),
            new InstantCommand(() -> IntakeShooter.getInstance().m_aimForDistance = true),
            Commands.waitUntil(() -> IntakeShooter.getInstance().intakeSequence == false).withTimeout(4),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterSpeedRatio(1)),
            Commands.waitUntil(() -> DriveTrain.getInstance().isStopped()).withTimeout(2),
            Commands.waitUntil(() -> IntakeShooter.getInstance().atTargetSpeed()).withTimeout(2),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(1)),
            Commands.waitUntil(() -> (Robot.isSimulation() ? true : IntakeShooter.getInstance().hasGamePiece()==false)).withTimeout(0.75),
             Commands.waitSeconds(0.5), 
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterSpeedRatio(0)),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(0)),
            new InstantCommand(() -> IntakeShooter.getInstance().m_aimForDistance = false),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up))
        ).finallyDo((val)->{
            // If sequence is interrupted or ended, stop autonomously aiming at speaker
            new InstantCommand(() -> IntakeShooter.getInstance().m_aimForDistance = false);
            Autonomous.getInstance().stopAiming();
            DriveTrain.getInstance().m_poseQueue.clear();
        });
    }
     
    public static Command ScoreAmp() {
        return Commands.sequence(
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up)),
            new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(PathPlanning.AprilTagAtDistance(ampTag(),0),Units.degreesToRadians(180))), // continuously aim at speaker
            new InstantCommand(() -> PathPlanning.getInstance().navigateTo( new Pose2d(
                                        PathPlanning.AprilTagAtDistance( ampTag(), AutoConstants.kAmpDistance+AutoConstants.kAmpDistanceInitial).getTranslation(),
                                        null) // Aiming is done by the aimAtPoint function above
                                    )),
            new InstantCommand(() -> Autonomous.getInstance().stopAiming()),
            new InstantCommand(() -> PathPlanning.getInstance().navigateTo( new Pose2d(
                                        PathPlanning.AprilTagAtDistance( ampTag(), AutoConstants.kAmpDistance).getTranslation(),
                                        new Rotation2d(Units.degreesToRadians(270)) // Aiming is done by the aimAtPoint function above
                                    ))),
            Commands.waitSeconds(1.5), // Wait for wheels to start turning
            Commands.waitUntil(() -> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,PathPlanning.AprilTagAtDistance(ampTag(),0),Units.feetToMeters(2),Math.PI)),
            Commands.waitUntil(() -> (DriveTrain.getInstance().m_poseQueue.isEmpty() || DriveTrain.getInstance().isStopped())).withTimeout(5),
            new InstantCommand(() -> DriveTrain.getInstance().m_poseQueue.clear()),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Amp)),
            Commands.waitUntil(() -> IntakeShooter.getInstance().intakeSequence == false).withTimeout(4),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterSpeedRatio(1)),
            Commands.waitUntil(() -> IntakeShooter.getInstance().atTargetAngle()).withTimeout(0.5),
            Commands.waitUntil(() -> IntakeShooter.getInstance().atTargetSpeed()).withTimeout(0.5),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(.75)),
            Commands.waitUntil(() -> IntakeShooter.getInstance().hasGamePiece() == false).withTimeout(1),
            Commands.waitSeconds(0.125),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterSpeedRatio(0)),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(0)),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up)),
            Commands.waitSeconds(0.125),
            new InstantCommand(() -> PathPlanning.getInstance().navigateTo( new Pose2d(
                                        PathPlanning.AprilTagAtDistance( ampTag(), AutoConstants.kAmpDistance+AutoConstants.kAmpDistanceInitial/2).getTranslation(),
                                        null)
                                    ))
        ).finallyDo((val)->{
            // If sequence is interrupted or ended, stop autonomously aiming at speaker
            Autonomous.getInstance().stopAiming();
            DriveTrain.getInstance().m_poseQueue.clear();
        });
    }

    private static Command nearNote(ArrayList<Pose2d> note, int index){
        return Commands.sequence(
            new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(note.get(index))),
            Commands.waitUntil(() -> DriveTrain.getInstance().atTargetHeading(Units.degreesToRadians(10))).withTimeout(3),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Ground)),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(0.75)),
            Commands.waitUntil(() -> IntakeShooter.getInstance().atTargetAngle()).withTimeout(3),
            new InstantCommand(() -> PathPlanning.getInstance().navigateCloseTo(note.get(index),AutoConstants.kNoteDistance,Math.PI))
        );
    }

    private static Command farNote(ArrayList<Pose2d> note, int index){
        return Commands.sequence(
            new InstantCommand(() -> Autonomous.getInstance().aimAtPoint(note.get(index))),
            new InstantCommand(() -> PathPlanning.getInstance().navigateCloseTo(note.get(index),AutoConstants.kNoteDistance,Math.PI)),
            Commands.waitUntil(() -> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,DriveTrain.getInstance().m_poseQueue.peekLast(),AutoConstants.kNoteNearDistance)).withTimeout(5),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Ground)),
            new InstantCommand(() -> IntakeShooter.getInstance().setIntakeSpeedRatio(0.75))
        );
    }

    public static Command PickupNote(ArrayList<Pose2d> note, int index){
         return Commands.sequence(
            nearNote(note, index).unless(
                ()-> !VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,note.get(index),AutoConstants.kNoteNearDistance)),
            farNote(note, index).unless(
                ()-> VectorUtils.isNear(PoseEstimator.getInstance().m_finalPose,note.get(index),AutoConstants.kNoteNearDistance)),
            Commands.waitUntil(() -> (IntakeShooter.getInstance().hasGamePiece())).withTimeout(3),
            new InstantCommand(() -> IntakeShooter.getInstance().setShooterAngle(ShooterConstants.kAnglePreset.Up))
        ).finallyDo((val)->{
            // If sequence is interrupted or ended, stop autonomously aiming at speaker
            Autonomous.getInstance().stopAiming();
            DriveTrain.getInstance().m_poseQueue.clear();
        });
    }

    public static Command NavToPose(Pose2d pose){
        return new InstantCommand(() -> PathPlanning.getInstance().navigateTo(pose));
    }

    static Pose2d m_storedPose;
    public static Command StorePose(){
        return new InstantCommand(() -> m_storedPose = PoseEstimator.getInstance().m_finalPose);
    }

    public static Command ReturnToPose(){
        return new InstantCommand(() -> {
                if (m_storedPose != null){
                    PathPlanning.getInstance().navigateTo(m_storedPose);
                }
            }
        );
    }
    
}
