clear; clc; close all;

a_siz=20;
array = zeros(1,a_siz);
array2 = zeros(1,a_siz);
x = linspace(1,a_siz,a_siz);

delete(instrfindall);
ports = seriallist;
port_exist = strfind(ports, "cu.usbserial");
idx = find(~cellfun(@isempty,port_exist));
current_port = ports(idx);

if ~isempty(current_port)
    try
        serial_p = serial(current_port,'BaudRate',115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none')
        fclose(serial_p);
        pause(0.2);
        fopen(serial_p);
        flag=1;
        fig = figure(1);
%         plt1 = plot(x, array, x, array2);
        hold on;
        hLine1 = plot(x,array, '-');
        hLine2 = plot(x,array2, '--');
        str = sprintf("Var=%.2f", 0.00);
        txt1 = text(a_siz*0.8,-170, str)
%         x = 1:length(array);
%         L ='';
%         [TF,L,U,C] = isoutlier(array, 'median');
%         hold on;
%         data_plt = plot(x,array);
%         TF_plt = plot(zeros(100), zeros(100), 'x');
%         L_plt = plot(x,L*ones(1,length(array)));
%         U_plt = plot(x,U*ones(1,length(array)));
%         C_plt = plot(x,C*ones(1,length(array)));
        ylim([-180 180]);
        
        
        while flag==1
            bytesAv = serial_p.BytesAvailable;
%             pause(0.001);
            
            %        str = fread(serial_p, 1, 'uchar')
            if bytesAv>0
                ser_string = fgets(serial_p);
                if (length(ser_string)>1)
                    start_brac = strfind(ser_string,"(");
                    end_brac = strfind(ser_string,")");
                    if ~(isempty(start_brac) && isempty(end_brac))
                        num_a_str = ser_string(start_brac(1)+1:end_brac(1)-1);
                        num_b_str = ser_string(start_brac(2)+1:end_brac(2)-1);
                        
                        num = str2double(num_a_str);
                        array = [array(2:end), 0];
                        array(end) = num;
                        
                        num2 = str2double(num_b_str);
                        array2 = [array2(2:end), 0];
                        array2(end) = num2;
                    end
                end
%                 if findstr(ser_string, "A")
%                     ser_string = ser_string(2:end);
%                     num = str2double(ser_string);
%                     array = [array(2:end), 0];
%                     array(end) = num;
%                 end
%                 if findstr(ser_string, "B")
%                     ser_string = ser_string(2:end);
%                     num2 = str2double(ser_string);
%                     array2 = [array2(2:end), 0];
%                     array2(end) = num2;
%                 end
%                 if findstr(ser_string, "C")
%                     ser_string = ser_string(2:end);
%                     num2 = str2double(ser_string);
%                     str = sprintf("Var=%.2f", num2);
%                     set(txt1, 'String', str);
% %                     fprintf("VAR:%.2f\n", num2);
%                 end
                
%                 [TF,L,U,C] = isoutlier(array, 'grubbs');
%                 
%                 set(data_plt, 'YData', array);
%                 set(TF_plt, 'XData', x(TF), 'YData', array(TF));
%                 
%                 set(L_plt, 'YData', L*ones(1,length(array)));
%                 set(C_plt, 'YData', C*ones(1,length(array)));
%                 set(U_plt, 'YData', U*ones(1,length(array)));
%                                 
                set(hLine1, 'YData', array);
                set(hLine2, 'YData', array2);
                                
%                 refreshdata(hAx,'caller')
                drawnow
            end
        end
        
    catch ME
        fprintf("line-%i, ME=%s\n", ME.stack.line, ME.message);
        
    end
    disp("Closing port.");
    fclose(serial_p);
    delete(serial_p)
    clear serial_p
else
    fprintf("Error opening port.\n");
end

