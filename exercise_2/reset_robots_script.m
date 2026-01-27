hudps = dsp.UDPSender('RemoteIPPort',1505);
hudps.RemoteIPAddress = '127.0.0.1';

% Default home position needed for the main to work
q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';

for t = 1:0.1:30
    disp(t);
    hudps([q; q]);   % <-- call object directly
    pause(1);
end
