function pack = recvData_sync(tcpObj)

if tcpObj.BytesAvailable >= 20
            tcpData = fread(tcpObj,5,'float32');
end

chunks = split(tcpData,'RM')
disp(chunks)
for ii = 1:length(chunks)
    if length(chunks(ii)) > 0
    end
end

pack = 0;
        