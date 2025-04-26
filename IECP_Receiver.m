clear all; close all;
Packet_Size = 62;
Polling_Frequency = 0.5;

Plot_Scale = 1.4;
Wait_Time = 0.05;
Plot_Span = 120;

TX_Buffer = zeros(Packet_Size, 1);
RX_Packet = [];
T = zeros(Packet_Size-2, 1);
E_Data = [0, 0]; %zeros(Packet_Size-2, 2);
current_time = 0;
current_step = 0;

pH_Data = [0, 0];
K_Data = [0, 0];
I_Raw = [0, 0];
I_Data = [0, 0];
D_Append = false;

Data_Table = [];
Write_Index = 1;
%{
Kill_Switch = uicontrol('Style','pushbutton', 'String', 'Kill', 'Units', ...
        'normalized','Position', [0.85 0.85 0.1 0.1], 'FontName', 'Arial',...
        'FontSize', 14, 'Callback', {@Kill_Command}, 'UserData', ...
        struct('Value', 0));
%}

Name_String = 'Pico';
BTable = blelist;
BT_Address = [];

Polling_Frequency = 0.2;

try
    for i = 1:size(BTable, 1)
        current_string = BTable{i, 2};
        if strfind(current_string, Name_String)
            BT_Address = BTable{i, 3};
        end
    end

    b = ble(BT_Address);
    c = characteristic(b, "Environmental Sensing", "Temperature");
    disp(strcat('BT connection established at address: ', BT_Address));

catch
    disp('Cannot find PICO in BLE list');
    return;
end

f = figure('Color', 'w', 'Units', 'inches', 'Position', [0.25 0.5 1.2*Plot_Scale*5.36 Plot_Scale*5.1025]);

ax1 = axes('Parent', f, 'Position', [0.1,0.1,0.8,0.2], 'LineWidth', 2.5*Plot_Scale);
S1 = scatter(K_Data(:, 1), K_Data(:, 2));
box on; ax1.LineWidth = 2.5*Plot_Scale; ax1.FontName = 'Arial'; 
ax1.FontSize = 12;
xlabel('Time (s)'); ylabel('V_{K} (V)');

ax2 = axes('Parent', f, 'Position', [0.1,0.4,0.8,0.2], 'LineWidth', 2.5*Plot_Scale);
S2 = scatter(pH_Data(:, 1), pH_Data(:, 2));
box on; ax2.LineWidth = 2.5*Plot_Scale; ax2.FontName = 'Arial'; 
ax2.FontSize = 12;
xlabel('Time (s)'); ylabel('V_{pH}');

ax3 = axes('Parent', f, 'Position', [0.1,0.7,0.4,0.2], 'LineWidth', 2.5*Plot_Scale);
S3 = plot(I_Raw(:, 1), I_Raw(:, 2), 'LineWidth', 2.5*Plot_Scale);
box on; ax3.LineWidth = 2.5*Plot_Scale; ax3.FontName = 'Arial'; 
ax3.FontSize = 12;
xlabel('Time (s)'); ylabel('I ({\mu}A)');

ax4 = axes('Parent', f, 'Position', [0.6,0.7,0.3,0.2], 'LineWidth', 2.5*Plot_Scale);
S4 = scatter(I_Data(:, 1), I_Data(:, 2));
box on; ax4.LineWidth = 2.5*Plot_Scale; ax4.FontName = 'Arial'; 
ax4.FontSize = 12;
xlabel('Time (s)'); ylabel('I_{T=10 s} ({\mu}A)');

while(1)

% Poll data packet from recording module    
    RX_Packet = read(c);
    TX_Buffer = Read_TX_Buffer(TX_Buffer, RX_Packet);
    T = linspace(TX_Buffer(2)/1000, TX_Buffer(2)/1000+30, 60)';
    
% Check if time index has changed and update current time value    
    if (~(current_time==TX_Buffer(2)))
        D_Append = true;
    else
        D_Append = false;
    end
    
% Convert ADC codes from AFE into decimal voltage or current values    
    if ((current_step == 0) | (current_step ==1))
        TX_Buffer = Calc_Voltage(TX_Buffer);
    else
        TX_Buffer = Calc_Current(TX_Buffer);
    end

% Handle potentiometric data
    if ((current_step == 0) | (current_step ==1))
        if (size(E_Data, 1) == 1); D_Append = true; end
        
        K_Temp = zeros(30, 2);
        pH_Temp = zeros(30, 2);
        
% Split Data into K and pH channels 
        for i = 3:size(TX_Buffer, 1)
            if mod(i, 2)
                K_Temp(floor((i-1)/2), 1) = T(i-2);
                K_Temp(floor((i-1)/2), 2) = TX_Buffer(i);
            else
                pH_Temp(floor((i-2)/2), 1) = T(i-2);
                pH_Temp(floor((i-2)/2), 2) = TX_Buffer(i);
            end
        end

% Append new data if timestamp has changed, otherwise update existing data
        if D_Append
            K_Data = [K_Data; [TX_Buffer(2)/1000 K_Data(end, 2)]];
            K_Data = [K_Data; K_Temp];

            pH_Data = [pH_Data; [TX_Buffer(2)/1000 pH_Data(end, 2)]];
            pH_Data = [pH_Data; pH_Temp];            
        else
            L = size(K_Data, 1);
            K_Data(L-29:L, :) = K_Temp;
            
            L = size(pH_Data, 1);
            pH_Data(L-29:L, :) = pH_Temp;
        end
                
% Update ax1 and ax2 with appropriate data
        set(S1, 'XData', K_Data(:, 1), 'YData', K_Data(:, 2)); drawnow;
        set(S2, 'XData', pH_Data(:, 1), 'YData', pH_Data(:, 2)); drawnow;
    
% Handle amperometric data    
    else
        if (size(I_Raw, 1) == 1); D_Append = true; end

        if (D_Append)
            I_Raw = [I_Raw; [TX_Buffer(2)/1000 I_Raw(end, 2)]];
            I_Raw = [I_Raw; [T TX_Buffer(3:end)]];
        else
            L = size(I_Raw, 1);
            I_Raw(L-59:L, :) = [T TX_Buffer(3:end)];
        end

        if (current_step == 4)
            if (size(I_Data, 1) == 1); D_Append = true; end
            if (D_Append)
                I_Data = [I_Data; [T(21) TX_Buffer(23)]];
            else
                I_Data(end, :) = [T(21) TX_Buffer(23)];
            end
        end

        set(S3, 'XData', I_Raw(:, 1), 'YData', I_Raw(:, 2)); drawnow;
        set(S4, 'XData', I_Data(:, 1), 'YData', I_Data(:, 2)); drawnow;

    end

    if (D_Append)
        E_Data = [E_Data; [TX_Buffer(2)/1000 I_Raw(end, 2)]];
        E_Data = [E_Data; [T TX_Buffer(3:end)]];
    else
        L = size(E_Data, 1);
        E_Data(L-59:L, :) = [T TX_Buffer(3:end)];
    end

% Save data to text every 60 seconds
    if (E_Data(end, 1) > 60*Write_Index)
        Data_Table = table(E_Data);
        writetable(Data_Table, 'Output_Data.txt');
        Write_Index = Write_Index + 1;
    end

    current_time = TX_Buffer(2);
    current_step = TX_Buffer(1);
    pause(1/Polling_Frequency);
end

function [TX_Buffer] = Read_TX_Buffer(TX_Buffer, RX_Packet)
    L = length(RX_Packet);
    wL = 4;
    BEnd_Hex_Data = {''};


    for i = 0:(size(RX_Packet, 2)/wL-1)
        BEnd_Hex_Data = {''};
        for j = 1:wL
            Hex_Data = dec2hex(RX_Packet(i*wL+(wL-j+1)));
            if size(Hex_Data, 2)==1
                Hex_Data = ['0', Hex_Data];
            end
            BEnd_Hex_Data = strcat(BEnd_Hex_Data, Hex_Data);
        end

    Num_Data = hex2dec(BEnd_Hex_Data);
    TX_Buffer(i+1) = Num_Data;
    end
end

function [TX_Buffer] = Calc_Voltage(TX_Buffer)
  kFactor = 1.835/1.82;
  fVolt = 0.0;
  tmp = 0;

  for i = 3:size(TX_Buffer, 1)
    if (~(TX_Buffer(i) == 0))
        tmp = TX_Buffer(i) - 32768;
        tmp = tmp/1.5;
        fVolt = tmp*1.82/32768*kFactor;
        TX_Buffer(i) = fVolt;
    end
  end
end

function [TX_Buffer] = Calc_Current(TX_Buffer)
  for i = 3:size(TX_Buffer, 1)
        TX_Buffer(i) = TX_Buffer(i)/1000;
  end
end

function Format_Plot(ax, Plot_Scale)
    set(ax, 'Color', 'none');
    set(ax, 'LineWidth', Plot_Scale*2.5);
    set(ax, 'XColor', [0, 0, 0]);
    set(ax, 'YColor', [0, 0, 0]);
    set(ax, 'FontName', 'Arial');
    set(ax, 'FontSize', Plot_Scale*15);
    set(ax, 'FontWeight', 'bold');
end
