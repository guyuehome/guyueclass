% SERVER Write a message over the specified port
% 
% Usage - server(message, output_port, number_of_retries)
function server(port, number_of_retries)

    import java.net.ServerSocket
    import java.io.*

    if (nargin < 2)
        number_of_retries = 2; % set to -1 for infinite
    end
    retry             = 0;

    server_socket  = [];
    output_socket  = [];
    
    port

    while true

        retry = retry + 1;

        try
            if ((number_of_retries > 0) && (retry > number_of_retries))
                fprintf(1, 'Too many retries\n');
                break;
            end

            fprintf(1, ['Try %d waiting for client to connect to this ' ...
                        'host on port : %d\n'], retry, port);

            % wait for 1 second for client to connect server socket
            server_socket = ServerSocket(port);
            server_socket.setSoTimeout(10000);
            
            fprintf(1, 'Waiting for client to connect\n');


            output_socket = server_socket.accept;

            fprintf(1, 'Client connected\n');

            input_stream   = output_socket.getInputStream;
            %d_output_stream = DataOutputStream(output_stream);

            for i=1:10
                % read data from the socket - wait a short time first
            pause(0.5);
            bytes_available = input_stream.available;
            fprintf(1, 'Reading %d bytes\n', bytes_available);
            end
            
            % clean up
            server_socket.close;
            output_socket.close;
            break;
            
        catch me
            me
            if ~isempty(server_socket)
                server_socket.close
            end

            if ~isempty(output_socket)
                output_socket.close
            end

            % pause before retrying
            pause(1);
   end
end
end
