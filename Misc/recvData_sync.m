function pack = recvData_sync(tcpObj)
pack = [];
tcpData_uint8 = [];
val = tcpObj.BytesAvailable;
if val > 0
    try
        tcpData_uint8 = fread(tcpObj,val,'char');
        tcpData_char = char(tcpData_uint8);
        tcpData_str = cellstr(tcpData_char');
        chunks = split(tcpData_str,'RM');
        lenC = length(chunks);
        for ii = 1:lenC
            %grab current chunk
            chunk = uint8(chunks{lenC-ii+1});
            %separate data and checksum
            data = chunk(1:end-2);
            binCheck = chunk(end-1:end);
            checksum = typecast(binCheck,'uint16');
            %calc checksum
            calcChecksum = cast(sum(data),'uint16');
            if calcChecksum == checksum
                %save data
                pack = typecast(data,'single');
                break;
            end
        end
    catch
%         disp('Read failed in recvData_sync')
%         if ~isempty(tcpData_uint8)
%             disp(tcpData_uint8)
%         end
    end        
end

        