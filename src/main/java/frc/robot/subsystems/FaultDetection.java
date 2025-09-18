// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FaultDetection extends SubsystemBase {

  LED m_led = LED.getInstance();
  DriveTrain m_DriveTrain = DriveTrain.getInstance();
  Gyro m_GyroSub = Gyro.getInstance();

  /** Creates a new FaultDetection. */
  public FaultDetection() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // int maxAMPS = 40;
    // boolean FMSconnect = DriverStation.isFMSAttached();
    // boolean DSconnect  = DriverStation.isDSAttached();
    // boolean JoyConnect = DriverStation.isJoystickConnected(0);
    // boolean JoyConnect1 = DriverStation.isJoystickConnected(1);
    // boolean disabled = DriverStation.isDisabled();

  
    //   if (!FMSconnect){
    //     //set the first group of LEDs to red
    //     m_led.setColor(LED.red);
    //   }
    //   if (!DSconnect){
    //     m_led.setColor(LED.red);
    //   }


    //   if(Math.abs(m_DriveTrain.headingError) > 10){
    //     m_led.setAltColors(LED.yellow, LED.blue);
    //   }
    //   if(!m_GyroSub.Gcalibration){
    //     m_led.setAltColors(LED.yellow, LED.orange);
    //   }
    //   if(!m_GyroSub.Ginitialized){
    //     m_led.setAltColors(LED.yellow, LED.green);
    //   }
    //   if(!m_GyroSub.Gsenorpresent){
    //     m_led.setAltColors(LED.yellow, LED.pink);
    //   }
      


    //   if (m_DriveTrain.MotorfrontLeft.getOutputCurrent() > maxAMPS){
    //     m_led.setAltColors(LED.red, LED.orange);
    //   }
    //   if (m_DriveTrain.MotorfrontRight.getOutputCurrent() > maxAMPS){
    //     m_led.setAltColors(LED.red, LED.orange);
    //   }
    //   if (m_DriveTrain.MotorrearLeft.getOutputCurrent() > maxAMPS){
    //     m_led.setAltColors(LED.red, LED.orange);
    //   }
    //   if (m_DriveTrain.MotorrearRight.getOutputCurrent() > maxAMPS){
    //     m_led.setAltColors(LED.red, LED.orange);
    //   }


    //   if (!JoyConnect){
    //     m_led.setAltColors(LED.orange, LED.blue);
    //   }
    //   if (!JoyConnect1){
    //     m_led.setAltColors(LED.orange, LED.blue);
    //   }

      

      
    
  }
}