% CLIENT connect to a server and read a message
%
% Usage - message = client(host, port, number_of_retries)
function message = client(host, port, number_of_retries)

    import java.net.Socket
    import java.io.*


    retry        = 0;
    input_socket = [];
    message      = [];

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
            input_socket = Socket(host, port);

            % get a buffered data input stream from the socket
            input_stream   = input_socket.getInputStream;
            d_input_stream = DataInputStream(input_stream);

            fprintf(1, 'Connected to server\n');

            % read data from the socket - wait a short time first
            for i=1:20
                pause(0.5);
                bytes_available = input_stream.available;
                %fprintf(1, 'Reading %d bytes\n', bytes_available);
                data_reader = DataReader(d_input_stream);
                message     = data_reader.readBuffer(bytes_available);
                
                message = char(message'); % Data comes out as a column vector
                message
            end
            
%             data_reader = DataReader(d_input_stream);
%         message     = data_reader.readBuffer(bytes_available);
%             
%             message = char(message'); % Data comes out as a column vector
            
            % cleanup
            input_socket.close;
            break;
            
        catch me
            me
            if ~isempty(input_socket)
                input_socket.close;
            end

            % pause before retrying
            pause(1);
   end
    end
end
