function sendData_sync(tcpObj, cmdPacket)

%flushinput(tcpObj);
fprintf(tcpObj,'%c','RM');
%fprintf(tcpObj, '%f', cmdPacket);
pack = cast(cmdPacket,'single');
fwrite(tcpObj, pack,'float');

binPack = typecast(pack, 'uint8');
checksum = cast(sum(binPack),'uint16');
fwrite(tcpObj, checksum,'uint16');
