// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Animate extends SubsystemBase {
  /** Creates a new Animate. */
  private ConnectorX cX;
  private ConnectorXAnimate cXAnimate;
  private boolean animateConnected;
  private boolean animateConnect;
  public Animate() {
    cX = new ConnectorX();
    
    
    animateConnected = cX.Connect(USBPort.kUSB1);
    
    SmartDashboard.putBoolean("xxx",animateConnected);
   
    cX.leds.SetColor("total", new Color(new Color8Bit(0,255,255)));

  }
  private void yes(){
    animateConnected = cX.Connect(USBPort.kUSB2);
   
    cX.leds.SetColor("total", new Color(new Color8Bit(255,0,0)));
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //yes();
  }
}
