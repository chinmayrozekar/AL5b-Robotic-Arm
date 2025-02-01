clear
% Create the new ArmRobot object
robot = ArmRobot('COM3');

% Set configuration of the robot
robot.setServoCenters([1330 1550 1770 1600 1640 1500]);
robot.setServoBounds([500 750 1000 700 750 1100],[2250 2050 2450 2450 2500 2500]);
robot.setLinkLengths([5 5 2 12 13.5 6 6.5]);

% Connect to the Robot
robot.connect();

display('Press Any Key to go to home position');
% 
% % % Step 0 - Home Position  ( was using bot 5 earlier now using bot 2)
robot.moveJoints([1300 1500 1700 1600 1600 1400],[0 1 2 3 4 5])
pause(1);
%     
% % Step 1 Repeatability
% for j = 0:141924798
%     robot.moveJoints(2400,3)
%     pause(1);
%     robot.moveJoints(2200,3)
%     pause(1);
% end

% display('Press Any Key for Step 2');
% pause();
% 
% % Step 2 Movejoint absolute
% display('Press Any Key for Step 3');
% robot.moveJoints([1300 1500 1700 1600 1600 1400],[0 1 2 3 4 5])
% pause(1);
% robot.moveAbsolute(30,2)
% pause(1);mo
% robot.moveAbsolute(30,3)



% % % Step 3 move Relative

% robot.moveRelative([30 45 50],[0 2 3])
% pause(1);
%robot.moveRelative([-30 -45 -50], [0 2 3])
%robot.moveRelative([])
%%switch to j+1 if code doesnt work

% display('Press Any Key for Step 4');
% pause();
% 
%  
% % Step 4 pseudo pick and place
for k = 0:455221
robot.moveJoints(1400,1)
 pause(1);
%  robot.moveJoints(1350,1)
%  pause(1);
 robot.moveJoints(900,3)
 pause(1);
 robot.moveJoints(2400,5)
 pause(1);
 robot.moveJoints(1500,1)
pause(1);
robot.moveJoints(1700,1)
pause(1);
robot.moveJoints(1500,0)
pause(1);
robot.moveJoints(1700,0)
pause(1);
robot.moveJoints(1600,1)
pause(1);
robot.moveJoints(1500,1)
pause(1);
robot.moveJoints(1400,1)
pause(1);
robot.moveJoints(1100,5)
pause(1);
robot.moveJoints([1300 1500 1700 1600 1600 1400],[0 1 2 3 4 5])
pause(1);
end



% display('Press Any Key for Step 5');
% pause();
%  
% % Step 5
% display('Press Any Key for Step 6');
% pause();
%  
% % Step 6
% display('Press Any Key to all steps absolute');
% pause();
% 
% % Now Lets Try that All At Once (Absolute)
% display('Press Any Key to all steps relative');
% pause();
% 
% % Now Lets Try that All At Once (Relative)
% display('Press Any Key to all steps relative linear');
% pause();
% 
% % Now Lets Try that SMOOTHLY All At Once (Relative Linear)
% display('Press Any Key to all steps absolute linear');
% pause();

% Close out the robot
%robot.delete();
