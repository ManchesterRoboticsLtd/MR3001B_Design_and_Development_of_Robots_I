<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Bco_Transparente.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/Logotipo%20Vertical%20Azul%20transparente.png">
  <img alt="Shows ITESM logo in black or white." width="160" align="right">
</picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/MCR2_Logo_White.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/blob/main/Misc/Logos/MCR2_Logo_Black.png">
  <img alt="Shows MCR2 logo in black or white." width="150" align="right">
</picture>

---
# MR3001B Design and Development of Robots I

  ## Introduction
* The student intends to solve a cyber-physical system challenge in this advanced-level engineering course.
* The course is divided into 2 sections, carefully designed for the user to learn about the different aspects of ROS,  from topics and messages to control and simulation of a real robot.
* The first part of this course, introduces the basic concepts and general knowledge of the ROS environment to the user.
* The second part of the course is dedicated to analyzing and controlling mobile robots in different environments. 
* This course will be based on challenges to inform students of ROS basics and ROS communication with hardware.
* This branch contains all the presentations, activities, and files required for the “MR3001C: Cyber-Physical Systems I” course of the Tec de Monterrey.
* This repository is organised by sessions. Each subfolder contains all the necessary files for each one of the activities of this course.
   
## General Information
* MCR2 Person in Charge: Dr Alexandru Stancu and Dr Mario Martinez
* Tecnológico de Monterrey Person in Charge: Dr Consuelo Rodríguez Padilla
* Duration: 10 Weeks
* Start Date: 7 August 2023


## Live Sessions (Recordings)
  * TBD
     
## General Requirements
General requirements. Please be aware that a set of requirements specific to each session will be shown in each session subsection (Some items may be repeated).
* Computer with access to Zoom (online classes).
* Computer with Ubuntu 18.04 and ROS Melodic or the Virtual Machine provided by MCR2.
* Knowledge of Windows. 
* Basic knowledge of Ubuntu (recommended).
* Basic understanding of robotics (recommended).
* Basic understanding of control (recommended).

## Sessions

### Session 1: Introduction
  This session will introduce the teaching team and the basics of ROS.
  #### Topics:
 * Who are we? 
 * Introduction to robotics.
 * Introduction to VM/Ubuntu
 * Introduction to ROS
 * Overview of ROS Environment
   * Topics, Messages
   * Launch Files

  #### Activities

   * ##### Activity 1: Talker and Listener
     Generate a node that sends a message to another node to listen to it.

   * ##### Activity 2: Launch Files
     Create a Launch file for the nodes created.

  #### Mini-Challenges

  * ##### Mini-Challenge 1: ROS Nodes
    Generate a node that sends a signal to another node to process it.
  
  **Requirements:** 
 * Computer with access to Zoom
 * Ubuntu 18.04 or 20 
 * ROS Melodic /Noetic Installed (Full installation).
 * If Ubuntu 18.04 or 22.04 cannot be installed, MCR2 offers a Virtual Machine with ROS preinstalled (installation instructions in Week 1 Folder).





### Session 2: ROS Practicalities
  This week will introduce some useful ROS practicalities.
  #### Topics:
 * ROS Namespaces
 * ROS Parameter Server
 * ROS Custom Messages


  #### Activities

   * ##### Activity 3: Parameters
     Parametrise previous nodes.

   * ##### Activity 4: Custom Messages
     Create a custom message for the previous nodes.

  #### Mini-Challenges

  * ##### Mini-Challenge 2: ROS Nodes
    P/PI Controller from scratch to a 1st order simulated system.  
  
  **Requirements:** 
    Requirements of Session 1.






### Session 3: ROS-Hardware Communication
  This week will introduce hardware communication between ROS and the Hackerboard/Arduino using ROS Serial.
  #### Topics:
  * ROS Serial
  * Arduino / ESP32 / Hackerboard
  * ROS Serial/Arduino Communication.

  #### Activities

   * ##### Activity 5: ROS Serial Communication
     Rosserial communication publisher with Arduino/ESP32.
     Rosserial communication subscriber with Arduino/ESP32.

  #### Mini-Challenges

  * ##### Mini-Challenge 3: ROS Nodes
    ROS communication with an Ultrasonic Sensor. 
  
  **Requirements:** 
  *	Requirements of Session 1.
  *	Installation of the Arduino IDE and the Rosserial package in the VM or Ubuntu (See instructions on Session2 MCR2_Arduino_IDE_Confirguration), 
  * Access to an Arduino or a Hackerboard and an HC-SRC04 Ultrasonic Sensor.

<picture>
  <source srcset="https://github.com/ManchesterRoboticsLtd/MR3001C_Cyber-Physical_Systems_I/assets/67285979/53196e95-1930-4e9a-9a29-ea66b2e6138b">
  <img alt="Shows Required Materials." width="630" align="center">
</picture>



  ---
## Declaration

At Manchester Robotics, we firmly believe that innovation is driven by change, so we have made it our mission to change access to educational robotics. We hope you enjoy our products and support this revolution.

So, from the team at MCR2, we would like to say 

                                                          Thank you!
                                                   {Learn, Create, Innovate};
---
  ## Disclaimer
 *THE PIECES, IMAGES, VIDEOS, DOCUMENTATION, ETC. SHOWN HERE ARE FOR INFORMATIVE PURPOSES ONLY. THE DESIGN IS PROPRIETARY AND CONFIDENTIAL TO MANCHESTER ROBOTICS LTD. (MCR2). THE INFORMATION, CODE, SIMULATORS, DRAWINGS, VIDEOS PRESENTATIONS ETC. CONTAINED IN THIS REPOSITORY IS THE SOLE PROPERTY OF MANCHESTER ROBOTICS LTD. ANY REPRODUCTION OR USAGE IN PART OR AS A WHOLE WITHOUT THE WRITTEN PERMISSION OF MANCHESTER ROBOTICS LTD. IS STRICTLY PROHIBITED*

*THIS WEBSITE MAY CONTAIN LINKS TO OTHER WEBSITES OR CONTENT BELONGING TO OR ORIGINATING FROM THIRD PARTIES OR LINKS TO WEBSITES AND FEATURES IN BANNERS OR OTHER ADVERTISING. SUCH EXTERNAL LINKS ARE NOT INVESTIGATED, MONITORED, OR CHECKED FOR ACCURACY, ADEQUACY, VALIDITY, RELIABILITY, AVAILABILITY OR COMPLETENESS BY US.*

*WE DO NOT WARRANT, ENDORSE, GUARANTEE, OR ASSUME RESPONSIBILITY FOR THE ACCURACY OR RELIABILITY OF ANY INFORMATION OFFERED BY THIRD-PARTY WEBSITES LINKED THROUGH THE SITE OR ANY WEBSITE OR FEATURE LINKED IN ANY BANNER OR OTHER ADVERTISING.*
