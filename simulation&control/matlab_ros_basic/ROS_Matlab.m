cmdpub = rospublisher('/turtle1/cmd_vel',rostype.geometry_msgs_Twist);
pause(3) % Wait to ensure publisher is setup
cmdmsg = rosmessage(cmdpub);
cmdmsg.Linear.X = 10;
cmdmsg.Angular.Z = 10;
send(cmdpub,cmdmsg)













% chatterpub = rospublisher('/chatter',rostype.std_msgs_String)
% pause(3) % Wait to ensure publisher is setup
% chattermsg = rosmessage(chatterpub);
% chattermsg.Data = 'hello world'
% send(chatterpub,chattermsg)
% pause(5)