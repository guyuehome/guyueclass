%SensorLog Receive data from SensorLog app
%
% Class to receive data from the SensorLog iPhone app
%
% S = SensorLogconnect to a server and read a message
%
% Usage - message = client(host, port, number_of_retries)
%
% Usage:
%  start the matlab code
%  start the server
% 

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

classdef SensorLog < handle
    properties
        socket
        stream
        d_stream
        struct
        names
        timer
    end
    
    properties (Hidden)
        cleanup
    end
    

    methods
        
        function s = SensorLog(host, port, timeout)
            


            import java.net.Socket
            import java.io.*
            
            
            retry        = 0;
            number_of_retries = 5;
            message      = [];
            
            if nargin < 3
                timeout = 30;
            end
            
            while true
                
                retry = retry + 1;
                if ((number_of_retries > 0) && (retry > number_of_retries))
                    fprintf(1, 'Too many retries\n');
                    break;
                end
                
                try
                    fprintf(1, 'Retry %d connecting to %s:%d\n', ...
                        retry, host, port);
                    
                    % throws if unable to connect
                    s.socket = Socket(host, port);
                    
                    % get a buffered data input stream from the socket
                    s.stream   = s.socket.getInputStream;
                    s.d_stream = DataInputStream(s.stream);
                    
                    fprintf(1, 'Connected to server\n');
                    
                    break;
                    
                    
                catch me
                    me
                    me.message
                    % pause before retrying
                    pause(1);
                end
            end
            
%             s.timer = timer('Period', 0.5, ...
%                 'TimerFcn', @timercb, ...
%                                 'UserData', s, ...
%                 'executionMode', 'fixedSpacing');
%             
%             start(s.timer)
        end
        

        function delete(s)
            if ~isempty(s.socket)
                s.socket.close;
            end
%             stop(s.timer)
%             set(s.timer, 'UserData', []);
%             delete(s.timer)

        end
        
        function flush(s)
        end
        
        function plot(s)
        end
        
        function msg = get(s)
            % read data from the socket - wait a short time first
            bytes_available = s.stream.available;
            %fprintf(1, 'Reading %d bytes\n', bytes_available);
            data_reader = DataReader(s.d_stream);
            msg = data_reader.readBuffer(bytes_available);
            
            msg = char(msg'); % Data comes out as a column vector
            
            for line = strsplit(msg, '\n')
                line = line{1};
                
                fields = strsplit(line, ',');
                if strcmp(fields{1}, 'time')
                    % have a header line
                    s.struct = [];
                    for i=1:length(fields)
                        s.struct = setfield(s.struct, fields{i}, 0);
                    end
                    fprintf('received header line\n');
                    s.names = fieldnames(s.struct)
                    continue;
                else
                    if ~isempty(s.names)
                    for i=1:length(fields)
                        s.struct = setfield(s.struct, s.names{i}, str2num(fields{i}));
                    end
                    end
                end
                
            end
            msg = s.struct;
        end
        
    end
end
function timercb(t, event)
    1
    %s = get(t, 'UserData');
    %s.get()
    
end

                
