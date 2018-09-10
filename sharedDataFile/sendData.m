function sendData(input,memLoc,size)
% Check size of input is not greater 'size'
len = length(input);
if len > size
    warning('in SendData(): input variable size larger than allocated size');
    input = input(1:size);  % Limit message to 255 characters.
    len = length(input); % Update len if str has been truncated. 
end
% Set first byte to zero, indicating a message is not yet ready.
memLoc.Data(1) = 0;
% Update the file via the memory map.
memLoc.Data(2:len+1) = input;
memLoc.Data(1)=len;

