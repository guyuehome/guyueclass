function connect_msg(port_handle)
prop = get(port_handle);
if isequal(prop.Line, -1)
    msg_str = 'Connection is broken!';
else
    msg_str = 'Connection is on!';
end
msgbox(msg_str, 'Connect');
end