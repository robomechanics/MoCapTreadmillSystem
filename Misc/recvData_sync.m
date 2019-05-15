function pack = recvData_sync(tcpObj)

if tcpObj.BytesAvailable > 0
    val = tcpObj.ValuesRevecied
    tcpData = fread(tcpObj,val,'float32');
end

chunks = split(tcpData,'RM')

for ii = 1:length(chunks)
    if length(chunks(ii)) > 0
    end
end

pack = 0;
        