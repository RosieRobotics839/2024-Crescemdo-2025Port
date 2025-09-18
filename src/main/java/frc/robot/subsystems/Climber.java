// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

public class Climber extends SubsystemBase {
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Climber");
  static NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/CAUTION/TestInput/Climber");

  static REVPhysicsSim sim = REVPhysicsSim.getInstance();

  private static Climber rightInstance = new Climber(ClimberConstants.kCANIDRight,"_right", true);
  private static Climber leftInstance = new Climber(ClimberConstants.kCANIDLeft, "_left", false);
  public static Climber right() {
    return rightInstance;
  }
  public static Climber left() {
    return leftInstance;
  }

  /** Convenience function to get average target height of climbers in meters. */
  public static double getAverageTargetHeight(){
    return (left().m_targetExtend+right().m_targetExtend)/2.0 + ClimberConstants.kMinHeight;
  }

  /** Convenience function to get average target height of climbers in meters. */
  public static double getTargetOffset(){
    return right().m_targetExtend-left().m_targetExtend;
  }
  
  /** Convenience function to offset the climbers while respecting max and min heights in meters.
   * An extended right climber is positive offset as it causes positive roll on the IMU */
  public static void setOffset(double offsetMeters){
    setRelativeHeight(getAverageTargetHeight(), offsetMeters);
  }
  
  /** Convenience function to modify the climber height while maintaining offset, respecting max and min heights in meters.*/
  public static void setAverageHeight(double heightMeters){
    setRelativeHeight(heightMeters, getTargetOffset());
  }

  /** Convenience function to offset the climbers while respecting max and min heights in meters. 
   * An extended right climber is positive offset as it causes positive roll on the IMU */
  public static void setRelativeHeight(double averageHeightMeters, double offsetMeters){
    double highestAvg = ClimberConstants.kMaxHeight-Math.abs(offsetMeters/2.0);
    double lowestAvg  = ClimberConstants.kMinHeight+Math.abs(offsetMeters/2.0);
    double limAverageHeight = Math.min(highestAvg, Math.max(lowestAvg, averageHeightMeters));

    left().setHeight(limAverageHeight-offsetMeters/2.0);
    right().setHeight(limAverageHeight+offsetMeters/2.0);
  }

  // The climber uses Canspark NEOs.

  // Static test variables to be set over network tables to control the climber system
  static NTDouble test_height = new NTDouble(Units.metersToInches(ClimberConstants.kMinHeight), testtable, "heightInch", val->setAverageHeight(Units.inchesToMeters(val)));
  static NTDouble test_offset = new NTDouble(0.0, testtable, "offsetInch", val->setOffset(Units.inchesToMeters(val)));

  public SparkMax m_climber;
  public RelativeEncoder m_encoderClimber;
  public SparkClosedLoopController m_pidClimber;

  DoublePublisher nt_target, nt_height, nt_i, nt_temp, nt_velocity;
  BooleanPublisher nt_init;

  double m_targetExtend;

  boolean m_init;
  double m_initLastMoveTime = 0;

  /** Creates a new Climber. */
  public Climber(int CANID, String name, Boolean isInverted) {
    m_climber = new SparkMax(CANID, MotorType.kBrushless);
    m_climber.setInverted(isInverted);
    m_encoderClimber = m_climber.getEncoder();
    m_pidClimber = m_climber.getClosedLoopController();
    m_pidClimber.setFeedbackDevice(m_encoderClimber);
    m_pidClimber.setP(ClimberConstants.kPIDKp);
    m_pidClimber.setI(ClimberConstants.kPIDKi);
    m_pidClimber.setD(ClimberConstants.kPIDKd);
    m_pidClimber.setFF(ClimberConstants.kPIDKff);
    m_pidClimber.setOutputRange(-1,1);
    m_pidClimber.setIZone(0.15);


    m_encoderClimber.setPositionConversionFactor(ClimberConstants.kEncoderPositionFactor);
    m_encoderClimber.setVelocityConversionFactor(ClimberConstants.kEncoderVelocityFactor);

    // Test variables set over network tables to control each climber side
    m_targetExtend = NTDouble.create(0, testtable,name+"/extensionInch", val->setExtension(Units.inchesToMeters(val)));
    m_init = NTBoolean.create(true, testtable, name+"/init", val->m_init=val);

    nt_init = testtable.getBooleanTopic(name+"/init").publish();
    nt_height = table.getDoubleTopic(name+"/heightInch").publish();
    nt_target = table.getDoubleTopic(name+"/targetInch").publish();
    nt_velocity = table.getDoubleTopic(name+"/targetIps").publish();
    nt_i = table.getDoubleTopic(name+"/currentA").publish();
    nt_temp = table.getDoubleTopic(name+"/degF").publish();

    if (Robot.isSimulation()){
      sim.addSparkMax(m_climber, (float)2.6, (float)5676.0);
    }

  }

  /** Sets the height in meters of the climber referenced from the floor. */
  public void setHeight(double heightMeters) {
    setExtension(heightMeters-ClimberConstants.kMinHeight);
  }

  /** Sets the extension in meters of the climber */
  public void setExtension(double extensionMeters) {
    m_targetExtend = Math.max(0,Math.min(extensionMeters,ClimberConstants.kMaxHeight-ClimberConstants.kMinHeight));
    m_pidClimber.setReference(m_targetExtend, SparkMax.ControlType.kPosition);
  }

  /** Gets the extension of the climber in meters. */
  public double getExtension(){
    return m_encoderClimber.getPosition();
  }

  /** Gets the height of the climber in meters from the floor. */
  public double getHeight(){
    return getExtension() + ClimberConstants.kMinHeight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_init && !RobotController.isSysActive()){
      m_initLastMoveTime = System.currentTimeMillis();
    }

    // Climber Initialization Retract with low current until it stops. Then set the relative encoder to 0.
    if (m_init && RobotController.isSysActive()){

      // Retract the climber with a current limit to avoid damaging anything.
      m_climber.setSmartCurrentLimit(ClimberConstants.kCurrentInit);
      m_pidClimber.setOutputRange(-ClimberConstants.kInitMaxEffort,ClimberConstants.kInitMaxEffort);
      m_pidClimber.setReference(-ClimberConstants.kMaxHeight, SparkMax.ControlType.kPosition);

      // If climber is still retracting, update the last move time with the current time
      if (m_climber.getOutputCurrent() < ClimberConstants.kInitCurrentLimit){
        m_initLastMoveTime = System.currentTimeMillis();
      }

      // If the climber hasn't moved in m_initTime, it is bottomed out..
      if (System.currentTimeMillis() > (m_initLastMoveTime + ClimberConstants.kInitTime*1.0e3)){
        m_init = false;
        nt_init.set(false);
        m_encoderClimber.setPosition(Units.inchesToMeters(-ClimberConstants.kInitRetractMargin)); // leave 1/8" margin to allow the rope to relax
        m_climber.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
        m_pidClimber.setReference(0, SparkMax.ControlType.kPosition);
        m_pidClimber.setOutputRange(-1, 1);
      }
    } 

    if (!m_init){
      m_pidClimber.setReference(m_targetExtend, SparkMax.ControlType.kPosition);
    };

    nt_height.set(Units.metersToInches(getHeight()));
    nt_target.set(Units.metersToInches(m_targetExtend));
    nt_velocity.set(Units.metersToInches(m_encoderClimber.getVelocity()));
    nt_i.set(m_climber.getOutputCurrent());
    nt_temp.set(m_climber.getMotorTemperature()*1.8+32);

  }
}
