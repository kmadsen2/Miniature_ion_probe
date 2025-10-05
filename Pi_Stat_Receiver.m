clear all; close all;

Kill_Switch = 0;
Plot_Scale = 0.5;

b = 0;
while(b==0)
    disp("Searching for Potentiostat");
    b = BLE_Connect;
end

Execution_Time = characteristic(b, "FF10", "FF11");
Current_State = characteristic(b, "FF10", "FF12");
Requested_State = characteristic(b, "FF10", "FF13");
K_Handle = characteristic(b, "FF10", "FF14");
pH_Handle = characteristic(b, "FF10", "FF15");
I_Handle = characteristic(b, "FF10", "FF16");
Cycle_Handle = characteristic(b, "FF10", "FF17");

t_Data = 0; %str2num(string(char(read(Execution_Time))));
K_Data = 0; %str2num(string(char(read(K_Handle))));
pH_Data = 0; %str2num(string(char(read(pH_Handle))));
I_Data = [0, 0]; %[t_Data str2num(string(char(read(I_Handle))))];

State_Values = 0;
Amperograms = {};
Calibration_Data = {};
Calibration_Data{1} = [];
Calibration_Data{2} = [];
Self_Test = [];
Amp_Lines = {};

% Main application function
Change_State(Requested_State, '0');
hFig = figure('Color', [0.1 0.1 0.1], 'Name', 'Ken_Stat_V2.0', 'NumberTitle', ...
    'off', 'Position', [0, 0, 800, 800]);

% Create axes for three plots
ax1 = axes('Parent', hFig, 'Position', [0.1, 0.8, 0.8, 0.175]); box on;
ax2 = axes('Parent', hFig, 'Position', [0.1, 0.6, 0.8, 0.175]); box on;
ax3 = axes('Parent', hFig, 'Position', [0.1, 0.4, 0.8, 0.175]); box on;
ax4 = axes('Parent', hFig, 'Position', [0.1, 0.1, 0.5, 0.2]); box on;

p1 = plot(ax1, t_Data, K_Data, 'LineWidth', 3*Plot_Scale, 'Color', 'r');
p2 = plot(ax2, t_Data, pH_Data, 'LineWidth', 3*Plot_Scale, 'Color', 'b');
p3 = scatter(ax3, I_Data(:, 1), I_Data(:, 2), 'LineWidth', 3*Plot_Scale, 'MarkerEdgeColor', [0.52 0.88 0.53]);
Amp_Lines{1} = plot(ax4, I_Data(:, 1), I_Data(:, 2), 'LineWidth', 3*Plot_Scale, 'Color', [0.52 0.88 0.53]);

Format_Plot(ax1, Plot_Scale); ax1.XTick = []; ax1.YLabel.String = 'V_{K}';
Format_Plot(ax2, Plot_Scale); ax2.XTick = []; ax2.YLabel.String = 'V_{pH}';
Format_Plot(ax3, Plot_Scale); ax3.YLabel.String = 'I_{t=10}';
ax3.XLabel.String = 'Time (s)';
Format_Plot(ax4, Plot_Scale); ax4.YLabel.String = 'I_{Lac}';
ax4.XLabel.String = 'Time (s)';

% Create buttons
btn1 = uicontrol('Style', 'pushbutton', 'String', 'Begin Measurement', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...                     
                 'Position', [0.7, 0.265, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '1'));

btn2 = uicontrol('Style', 'pushbutton', 'String', 'Pause Measurement', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...                     
                 'Position', [0.7, 0.225, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '0'));

btn3 = uicontrol('Style', 'pushbutton', 'String', 'Instant Amp', ...
                 'FontName', 'Arial', 'FontSize', 10, ...
                 'Units', 'normalized', ...
                 'Position', [0.7, 0.185, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '2'));

btn4 = uicontrol('Style', 'pushbutton', 'String', 'Calibrate Probe', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...
                 'Position', [0.7, 0.145, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '3'));

btn5 = uicontrol('Style', 'pushbutton', 'String', 'Self Test', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...
                 'Position', [0.7, 0.105, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '4'));

btn6 = uicontrol('Style', 'pushbutton', 'String', 'HW Reset', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...
                 'Position', [0.7, 0.065, 0.2, 0.03], ...
                 'Callback', @(src, event) Change_State(Requested_State, '5'));

btn7 = uicontrol('Style', 'pushbutton', 'String', 'End Measurement', ...
                 'Units', 'normalized', ...
                 'FontName', 'Arial', 'FontSize', 10, ...
                 'Position', [0.7, 0.025, 0.2, 0.03], ...
                 'Callback', @Kill_Now);

p1.XData = t_Data; p1.YData = K_Data;
p2.XData = t_Data; p2.YData = pH_Data;
p3.XData = I_Data(:, 1); p3.YData = I_Data(:, 2);

axes(ax3); box on;
drawnow;

while(str2num(string(char(read(Current_State))))==0)
    drawnow;
    pause(0.5);
end

Last_Count = str2num(string(char(read(Execution_Time))));
Cycle_Number = str2num(string(char(read(Cycle_Handle))));
Previous_Cycle = Cycle_Number;

%% Main loop %%
while(1)
    State_Values = [State_Values; str2num(string(char(read(Current_State))))];
    Cycle_Number = str2num(string(char(read(Cycle_Handle))));
    Current_Time = str2num(string(char(read(Execution_Time))));

    D1 = 0; D2 = 0;

    if (~(Last_Count == Current_Time))
        switch State_Values(end)
            case 0

            case 1
                if (Cycle_Number==3 && Previous_Cycle==2)
                    Amperograms{end+1} = [];
                end
                
                if (Cycle_Number==0 && Previous_Cycle==3)
                    [~, xst] = min(abs(Amperograms{end}(:, 1) - Amperograms{end}(1, 1) - 10));                   
                    I_Data = [I_Data ; Amperograms{end}(xst, :)];

                    axes(ax4); hold on;
                    
                    Amp_Lines{end+1} = plot(Amperograms{end}(:, 1)-Amperograms{end}(1, 1), Amperograms{end}(:, 2), 'LineWidth', 3*Plot_Scale);
                    jetcustom = jet(size(Amp_Lines, 2));
                    for i = 1:size(Amp_Lines, 2)
                        Amp_Lines{i}.Color = jetcustom(i, :);
                    end
                    
                    hold off;

                end

                switch Cycle_Number(end)
                    case 0
                        t_Data = [t_Data; Current_Time];
                        K_Data = [K_Data; str2num(string(char(read(K_Handle))))];
                        pH_Data = [pH_Data; str2num(string(char(read(pH_Handle))))];
                        ax1.XLim(2) = t_Data(end);
                    case 2
                        t_Data = [t_Data; Current_Time];
                        K_Data = [K_Data; str2num(string(char(read(K_Handle))))];
                        pH_Data = [pH_Data; str2num(string(char(read(pH_Handle))))];
                        ax1.XLim(2) = t_Data(end);
                    case 3
                        temp = [Current_Time str2num(string(char(read(I_Handle))))];
                        Amperograms{end} = [Amperograms{end}; temp];
                end

            case 2
                if (Cycle_Number==3 && Previous_Cycle==2)
                    Amperograms{end+1} = [];
                end
                
                if (Cycle_Number==0 && Previous_Cycle==3)
                    [~, xst] = min(abs(Amperograms{end}(:, 1) - Amperograms{end}(1, 1) -10));                   
                    I_Data = [I_Data ; Amperograms{end}(xst, :)];

                    axes(ax4); hold on;
                    plot(Amperograms{end}(:, 1)-Amperograms{end}(1, 1), Amperograms{end}(:, 2), 'LineWidth', 3*Plot_Scale);
                    hold off;                    
                end

                switch Cycle_Number(end)
                    case 0
                        t_Data = [t_Data; Current_Time];
                        K_Data = [K_Data; str2num(string(char(read(K_Handle))))];
                        pH_Data = [pH_Data; str2num(string(char(read(pH_Handle))))];                   
                        ax1.XLim(2) = t_Data(end);
                    case 2
                        t_Data = [t_Data; Current_Time];
                        K_Data = [K_Data; str2num(string(char(read(K_Handle))))];
                        pH_Data = [pH_Data; str2num(string(char(read(pH_Handle))))];
                        ax1.XLim(2) = t_Data(end);
                    case 3
                        temp = [Current_Time str2num(string(char(read(I_Handle))))];
                        Amperograms{end} = [Amperograms{end}; temp];
                end

            case 3
                if (Cycle_Number==3 && Previous_Cycle==2)
                    Amp_Cal = [];
                end
                
                if (Cycle_Number==0 && Previous_Cycle==3)
                    [~, xst] = min(abs(Amp_Cal(:, 1) - Amp_Cal(1, 1) - 10));                   
                    I_Data = [I_Data ; Amp_Cal(xst, :)];

                    Calibration_Data{2} = Amp_Cal;
                end

                switch Cycle_Number(end)
                    case 0
                        temp = [Current_Time, str2num(string(char(read(K_Handle)))), ...
                                str2num(string(char(read(pH_Handle))))];
                        Calibration_Data{1} = [Calibration_Data{1}; temp];
                
                    case 3
                        temp = [Current_Time str2num(string(char(read(I_Handle))))];
                        Amp_Cal = [Amp_Cal; temp];
                end
            case 4
                t_Data = [t_Data; Current_Time];
                K_Data = [K_Data; str2num(string(char(read(K_Handle))))];
                pH_Data = [pH_Data; str2num(string(char(read(pH_Handle))))];   
                ax1.XLim(2) = t_Data(end);

                temp = [Current_Time K_Data(end)];
                Self_Test = [Self_Test; temp];
        end

        p1.XData = t_Data; p1.YData = K_Data;
        p2.XData = t_Data; p2.YData = pH_Data;
        p3.XData = I_Data(:, 1); p3.YData = I_Data(:, 2);
        
        if (size(t_Data, 1)>2)
            ax1.XLim(1) = t_Data(2);
        end
        ax2.XLim = ax1.XLim;
        ax3.XLim = ax1.XLim;

        ax1.YLim = [-0.2 0.5];
        ax2.YLim = [-0.2 0.5];
        ax3.YLim = [-5 0];
        ax4.YLim = [-15 0];

        Last_Count = Current_Time;
        drawnow;
        Previous_Cycle = Cycle_Number;
    end


    % Kill State
    if (Kill_Switch == 1)
        Change_State(Requested_State, '0');
        outputToCSV(t_Data, K_Data, pH_Data, State_Values, Amperograms,...
                    I_Data, Calibration_Data, Self_Test);
        return;
    end
end



if (Kill_Switch == 1)
    Change_State(Requested_State, '0');
    return;
end

%% Functions %%
function Change_State(Requested_State, State)
    drawnow;
    write(Requested_State, State);
end

function Kill_Now(~, ~)
    assignin('base', 'Kill_Switch', 1);
end

function outputToCSV(t_Data, K_Data, pH_Data, State_Values, Amperograms, I_Data, Calibration_Data, Self_Test)
    Data = [t_Data K_Data pH_Data];
    Data_Dir = uigetdir;
    if ~Data_Dir
        return
    end
    Data_Dir = strcat(Data_Dir, '\Pi_Stat_Data.xlsx');
    % Write data to CSV file
    writematrix(Data, Data_Dir, 'Sheet', 'T K pH');
    %writecell(Amperograms, Data_Dir, 'Sheet', 2);
    ct = 1;
    for i = 1:size(Amperograms, 2)
        sz = size(Amperograms{i});
        SLetter = num2xlcol(ct);
        ct = ct+1;
        ELetter = num2xlcol(ct);
        Data_Range = strcat(SLetter, num2str(1), ':', ELetter, num2str(sz(1)));
        ct = ct+1;
        writematrix(Amperograms{i}, Data_Dir, 'Sheet', 'Amperograms', 'Range', Data_Range);
    end

    writematrix(I_Data, Data_Dir, 'Sheet', 'I T = 10');
    writematrix(Calibration_Data{1}, Data_Dir, 'Sheet', 'Single Point Calibration K pH');
    writematrix(Calibration_Data{2}, Data_Dir, 'Sheet', 'Single Point Calibration I');
    writematrix(Self_Test, Data_Dir, 'Sheet', 'Self Test Results');
end

function Format_Plot(ax, Plot_Scale)
    box on;
    set(ax, 'Color', 'none');
    set(ax, 'LineWidth', Plot_Scale*2.5);
    set(ax, 'XColor', 'w');
    set(ax, 'YColor', 'w');
    set(ax, 'FontName', 'Arial');
    set(ax, 'FontSize', Plot_Scale*30);
    set(ax, 'FontWeight', 'bold');
end

function xlcol_addr=num2xlcol(col_num)
% col_num - positive integer greater than zero
    n=1;
    while col_num>26*(26^n-1)/25
        n=n+1;
    end
    base_26=zeros(1,n);
    tmp_var=-1+col_num-26*(26^(n-1)-1)/25;
    for k=1:n
        divisor=26^(n-k);
        remainder=mod(tmp_var,divisor);
        base_26(k)=65+(tmp_var-remainder)/divisor;
        tmp_var=remainder;
    end
    xlcol_addr=char(base_26); % Character vector of xlcol address
end

function [b] = BLE_Connect
    Name_String = 'Pico';
    BTable = blelist;
    BT_Address = [];

    try
        for i = 1:size(BTable, 1)
            current_string = BTable{i, 2};
            if strfind(current_string, Name_String)
                BT_Address = BTable{i, 3};
            end
        end
    
        b = ble(BT_Address);
        disp(strcat('BT connection established at address: ', BT_Address));
    catch
        b = 0;
        return;
    end
end
