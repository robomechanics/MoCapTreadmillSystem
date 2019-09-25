# MoCapTreadmillSystem
Project Summary:
This project was to create an automated gait optimization system using our existing Optitrack Motion Capture (MoCap) cameras and Bertec treadmill. The MoCap system provides the location and orientation of the robot and Matlab GUI’s are used to run the optimization and control of the treadmill. The type of optimization routine currently used in this system is a modified version of Matlab’s fminsearch function which uses Neadler-Mead’s Simplex method to optimize the gait parameters in real time.

Install Instructions:
1)	Download Github repo onto computer running the MoCap and treadmill programs.
a.	Versions used with current code base:
i.	Optitrack Motive 10
ii.	Bertec Treadmill Control Panel 1.7.46
2)	Download NatNet SDK and put it in your MoCapTreadmillSystem repo folder (needed to run interface between Optirack Motive and Matlab) : https://optitrack.com/downloads/developer-tools.html (NatNet SDK 2.10 under Previous Releases)
3)	Add MoCapTreadmillSystem folder and all subfolders to your Matlab path (detailed in run instruction)

Run Instructions: https://github.com/robomechanics/MoCapTreadmillSystem/blob/master/Run%20Instructions%20for%20Automated%20Gait%20Optimization%20System.docx
