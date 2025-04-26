clear all; close all;
%% Simulation Parameters
Starting_Concentration = false;
Reduction = false;

H2O2_Input = 'H2O2_Concentration_600s_1mM_Menten.mat';
Lactate_Input = 'Lactate_Concentration_600s_5mM_r.mat';

H2O2_Output = 'H2O2_Concentration_600s_1mM_Menten';
Lactate_Output = 'Lactate_Concentration_600s_1mM_Menten';

% Timing:
Start_Time = 0; End_Time = 600; N_Points = 6000;
Time = linspace(Start_Time, End_Time, N_Points);

% Concentrations:
H2O2_Bulk = 0; Lactate_Bulk = 1E-3;

% Diffusion Coefficients:
H2O2_Diff = 1.9E-3; % cm^2/s Communication—Hydrogen Peroxide Re/duction in Aqueous 
% Electrolytes: Influence of a Heterogeneous Decomposition Step

Lactate_Diff = 100*(10000*0.963)/(1E9); % cm^2/s Binary Diffusion Coefficients
% for Aqueous Solutions of Lactic Acid

Membrane_Diffusion = H2O2_Diff;
Membrane_Thickness = 0.1;
Electrode_Height = 0.0375;

Max_Velocity = .005; Menten_Coeff = 0.0026; %0.0026;

Voxel_Size = 0.05; F = 96485;
V_Volume = (Voxel_Size/100)^3; 
Diffusion_Length = 10*sqrt(2*H2O2_Diff*(End_Time-Start_Time)/N_Points);
Color_Limit = [0, 1.5E-3];
C_Threshold = 1;
Max_Height = 3;


%% Output options
Replot = false;
Write_Video = true; 
Cross_Section = true; 
CS_Direction = 'y';
CS_Position = -6; F_CS = {};
Display_Figure = true;
Snapshot_Spread = 6;
CS_Spread = 99;
n_snapshots = floor(N_Points/Snapshot_Spread);
Frame_Length = 5;
Store_Data = false;
Save_Final = true;
Plot_Scale = 1.5;

X_Slice_Number = 10;
Y_Slice_Number = 0;
Z_Slice_Number = 0;

X_Slices = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9];
Y_Slices = [];
Z_Slices = [];


View_Angle = [45, 30]; 
View_Angle = [0, 90]; 
View_Angle = [-45, 45];
View_Angle = [-7, 5];
View_Angle = [-30, 15];
View_Angle = [-15 10];
%View_Angle = [-75 30];

X_Limits = [];
X_Limits = [-1 1];
Y_Limits = [];
%Y_Limits = [-9.5 -3.5];
Z_Limits = [];
Z_Limits = [-1 1];

%{
X_Limits = [-1.5 1.5];
Y_Limits = [-9.5 -3];
Z_Limits = [-2 2];
%}

%% Import autocad file
disp('Select electrode dxf file.');
[E_Filename, E_Filedir] = uigetfile('*.*',  'All Files (*.*)');
if E_Filename == 0; return; end
E_Filename = fullfile(E_Filedir, E_Filename);

DXF = DXFtool(E_Filename); close all;
DXF_Points = {};
DXF_Limits = [];
Layers = {'Layers'};
for i = 1:size(DXF.entities, 2)
    DXF_Points{i} = [DXF.entities(i).poly(:, 1) DXF.entities(i).poly(:, 2)];

    DXF_Limits = [DXF_Limits; [min(DXF_Points{i}(:, 1)),...
        max(DXF_Points{i}(:, 1)), min(DXF_Points{i}(:, 2)), ...
        max(DXF_Points{i}(:, 2))]];

    if ~isequal(DXF.entities(i).layer, Layers{end})
        Layers{end+1} = DXF.entities(i).layer;
    end
end

Boundaries = [min(DXF_Limits(:, 1)) max(DXF_Limits(:, 2));...
              min(DXF_Limits(:, 3)) max(DXF_Limits(:, 4))];

%% Import ply volume file
disp('Select encapsulation geometry file.');
[V_Filename, V_Filedir] = uigetfile('*.*',  'All Files (*.*)');
if V_Filename == 0; return; end
V_Filename = fullfile(V_Filedir, V_Filename);

Volume = plyread(V_Filename);

if isempty(Max_Height)
    Boundaries = [Boundaries; [min(Volume.vertex.z) max(Volume.vertex.z)]];
else
    Boundaries = [Boundaries; [min(Volume.vertex.z) Max_Height]];
end

H2O2_Dcoeffs = ones(uint16((Boundaries(1, 2) - Boundaries(1, 1))/Voxel_Size),...
          uint16((Boundaries(2, 2) - Boundaries(2, 1))/Voxel_Size),...
          uint16((Boundaries(3, 2) - Boundaries(3, 1))/Voxel_Size));
Lactate_Dcoeffs = H2O2_Dcoeffs;

 x = linspace(Boundaries(1, 1), Boundaries(1, 2), size(H2O2_Dcoeffs, 1));
 y = linspace(Boundaries(2, 1), Boundaries(2, 2), size(H2O2_Dcoeffs, 2));
 z = linspace(Boundaries(3, 1), Boundaries(3, 2), size(H2O2_Dcoeffs, 3));

 Mesh_List = [];
 for i = 1:length(x)
     for j = 1:length(y)
         Mesh_List = [Mesh_List; [x(i), y(j), i, j]];
     end
 end

 H2O2_Dcoeffs = H2O2_Diff*Define_Dcoeffs(Mesh_List, x, y, z, H2O2_Dcoeffs, Volume, 0);
 Lactate_Dcoeffs = Lactate_Diff*Define_Dcoeffs(Mesh_List, x, y, z, Lactate_Dcoeffs, Volume, 0);

 Discrete_LDiff = uint16((Boundaries(3, 2)-Boundaries(3, 1))/Diffusion_Length);
 
%% Set up initial concentration profile and map diffusion coefficients
H2O2_Concentration = H2O2_Bulk*ones(size(H2O2_Dcoeffs));
Lactate_Concentration = Lactate_Bulk*ones(size(H2O2_Dcoeffs));

H2O2_Concentration = Define_Dcoeffs(Mesh_List, x, y, z, H2O2_Concentration, Volume, 0);
Lactate_Concentration = Define_Dcoeffs(Mesh_List, x, y, z, Lactate_Concentration, Volume, 0);

[~, Electrode_Height_Index] = min(abs(z-Electrode_Height));

[~, Membrane_Height_Index] = min(abs(z-(Electrode_Height+Membrane_Thickness)));

in = {}; DXF_Maps = {};
for i = 1:size(DXF_Points, 2)
    DXF_Maps{i} = zeros(length(x), length(y));
    in{i} = inpolygon(Mesh_List(:, 1), Mesh_List(:, 2), DXF_Points{i}(:, 1), DXF_Points{i}(:, 2));
    for j = 1:length(in{i})
        DXF_Maps{i}(Mesh_List(j, 3), Mesh_List(j, 4)) = in{i}(j);
    end
end

% Define Membrane points
Membrane_Layer = find(contains(Layers, 'Lox'))-1;
[row, col] = find(DXF_Maps{Membrane_Layer});
Membrane_Points = [row col];

for i = 1:size(Membrane_Points, 1)
    Lactate_Dcoeffs(Membrane_Points(i, 1), Membrane_Points(i, 2), ...
                    Electrode_Height_Index:Membrane_Height_Index) = Membrane_Diffusion;

    H2O2_Dcoeffs(Membrane_Points(i, 1), Membrane_Points(i, 2), ...
                    Electrode_Height_Index:Membrane_Height_Index) = Membrane_Diffusion;
end

H2O2_Dcoeffs = cat(3, flip(H2O2_Dcoeffs, 3), H2O2_Dcoeffs);
Lactate_Dcoeffs = cat(3, flip(Lactate_Dcoeffs, 3), Lactate_Dcoeffs);

H2O2_Concentration = cat(3, flip(H2O2_Concentration, 3), H2O2_Concentration);
Lactate_Concentration = cat(3, flip(Lactate_Concentration, 3), Lactate_Concentration);

if Starting_Concentration
    H2O2_Concentration = load(H2O2_Input);
    Lactate_Concentration = load(Lactate_Input);
    H2O2_Concentration = H2O2_Concentration.H2O2_Concentration;
    Lactate_Concentration = Lactate_Concentration.Lactate_Concentration;
end

if isempty(Max_Height)
    Boundaries(3, 1) = -1*Boundaries(3, 2);
else
    Boundaries(3, 1) = -1*Max_Height;
end

z = linspace(Boundaries(3, 1), Boundaries(3, 2), size(H2O2_Dcoeffs, 3));

[~, Electrode_Height_Index] = min(abs(z-Electrode_Height));
[~, Membrane_Height_Index] = min(abs(z-(Electrode_Height+Membrane_Thickness)));

 Aspect_Ratio = [Boundaries(1, 2)-Boundaries(1, 1), ...
                 Boundaries(2, 2)-Boundaries(2, 1), ...
                 Boundaries(3, 2) - Boundaries(3, 1)];
             
 Membrane_X_Midpoint = uint16(mean(Membrane_Points(:, 1)));
 Membrane_Y_Midpoint = uint16(mean(Membrane_Points(:, 2)));
 
 % Define Cathode points
Cathode_Layer = find(contains(Layers, 'Cathode'))-1;
[row, col] = find(DXF_Maps{Cathode_Layer});
Cathode_Points = [row col];
Cathode_Points = [Cathode_Points (Electrode_Height_Index+1)*ones(length(row), 1)];

% Define Anode points
Anode_Layer = find(contains(Layers, 'Anode'))-1;
[row, col] = find(DXF_Maps{Anode_Layer});
Anode_Points = [row col];

%% Compute initial concentration gradients and starting mass balance 
Total_Lactate = sum(Lactate_Concentration, "all");
Total_H2O2 = sum(H2O2_Concentration, "all");

H2O2_Snapshot = {H2O2_Concentration};
Lactate_Snapshot = {Lactate_Concentration};

D_H2O2 = dCdT(H2O2_Concentration, H2O2_Dcoeffs, Voxel_Size); 
D_Lactate = dCdT(Lactate_Concentration, Lactate_Dcoeffs, Voxel_Size); 

%% Initialize plot
Snapshot_Counter = 0;
CS_Counter = 0;
Time_Points = 0;
Charge = 0;

if Display_Figure
    f1 = figure('Color', 'w', 'Units', 'inches', 'Position', [0.25 1 Plot_Scale*5.36 Plot_Scale*4.1025]);
    f2 = figure('Color', 'w', 'Units', 'inches', 'Position', [8.25 1 Plot_Scale*5.36 Plot_Scale*4.1025]);
    
    if X_Slice_Number > 0 && isempty(X_Slices)
        X_Slices = uint16(linspace(1, length(x), X_Slice_Number + 2));
        X_Slices = X_Slices(2:end-1); X_Slices = x(X_Slices);
    elseif X_Slice_Number == 0 && isempty(X_Slices)
        X_Slices = [];
    end
    
    if Y_Slice_Number > 0 && isempty(Y_Slices)
        Y_Slices = uint16(linspace(1, length(y), Y_Slice_Number + 2));
        Y_Slices = Y_Slices(2:end-1); Y_Slices = y(Y_Slices);
    elseif Y_Slice_Number == 0 && isempty(Y_Slices)
        Y_Slices = [];
    end
    
    if Z_Slice_Number > 0 && isempty(Z_Slices)
        Z_Slices = uint16(linspace(1, length(z), Z_Slice_Number + 2));
        Z_Slices = Z_Slices(2:end-1); Z_Slices = z(Z_Slices);
    elseif Z_Slice_Number == 0 && isempty(Z_Slices)
        Z_Slices = [];
    end
    
    Slice_Plot = slice(x, y, z, permute(H2O2_Concentration, [2, 1, 3]),...
                       X_Slices, Y_Slices, Z_Slices);
    
    [f1, Slice_Plot] = update_plot(f1, H2O2_Concentration, Volume, Slice_Plot, x, y, z,...
    X_Slices, Y_Slices, Z_Slices, Aspect_Ratio, View_Angle, X_Limits, ...
    Y_Limits, Z_Limits, Boundaries, Color_Limit, 1, Cross_Section, ... 
    CS_Direction, CS_Position);

    set(0, 'currentfigure', f2);
    Slice_Plot_Lactate = slice(x, y, z, permute(Lactate_Concentration, [2, 1, 3]),...
                   X_Slices, Y_Slices, Z_Slices);

    [f2, Slice_Plot_Lactate] = update_plot(f2, Lactate_Concentration, Volume, Slice_Plot, x, y, z,...
    X_Slices, Y_Slices, Z_Slices, Aspect_Ratio, View_Angle, X_Limits, ...
    Y_Limits, Z_Limits, Boundaries, Color_Limit, 0, Cross_Section, ... 
    CS_Direction, CS_Position);
    
    if Replot && Starting_Concentration
        if Cross_Section
            if strcmp(CS_Direction, 'x')
                [~, S_Index] = min(abs(x - CS_Position));
                C_Section =  reshape(H2O2_Concentration(S_Index, :, :), [length(y), length(z)]);
            elseif strcmp(CS_Direction, 'y')
                [~, S_Index] = min(abs(y - CS_Position));
                C_Section =  reshape(H2O2_Concentration(:, S_Index, :), [length(x), length(z)]);
            elseif strcmp(CS_Direction, 'z')
                [~, S_Index] = min(abs(z - CS_Position));
                C_Section =  reshape(H2O2_Concentration(:, :, S_Index), [length(x), length(y)]);            
            else
                disp('Invalid Slice Direction');
            end

            F_CS{end+1} = figure('Color', 'w', 'Units', 'inches', 'Position', [0.25 1 7.5 5.74]);
            F_CS = plot_CS(F_CS, x, y, z, CS_Direction, C_Section, ...
                Aspect_Ratio, X_Limits, Y_Limits, Z_Limits, Boundaries, ...
                Color_Limit, Plot_Scale);
        end
        return
    end
end

if Write_Video    
    Out_Frame = struct('cdata',[],'colormap',[]);
    Out_Frame_Lactate = struct('cdata',[],'colormap',[]);
    
    set(0, 'currentfigure', f1);
    drawnow
    Out_Frame = getframe(f1);

    set(0, 'currentfigure', f2);
    drawnow
    Out_Frame_Lactate = getframe(f2);
end

H2O2_Point_Concentrations = reshape(H2O2_Concentration(Membrane_X_Midpoint, Membrane_Y_Midpoint, :), [length(z), 1]);
Lactate_Point_Concentrations = reshape(Lactate_Concentration(Membrane_X_Midpoint, Membrane_Y_Midpoint, :), [length(z), 1]);


Waitbar = waitbar(0, 'Hold on to your butts');
%%Begin Iteration
for i = 2:length(Time)
    dt = Time(i)-Time(i-1);
    
    [H2O2_Concentration, Lactate_Concentration] = Menten(H2O2_Concentration, ...
    Lactate_Concentration, Cathode_Points, Membrane_Points, ...
    Electrode_Height_Index, Membrane_Height_Index, dt, Menten_Coeff, ...
    Max_Velocity);
    
    [H2O2_Concentration, Lactate_Concentration, Mass_Peroxide] = Bdry_Conditions(H2O2_Concentration, ...
    Lactate_Concentration, Cathode_Points, Anode_Points, Membrane_Points,...
    Electrode_Height_Index, Membrane_Height_Index, H2O2_Bulk, Lactate_Bulk, Reduction);
    Charge = [Charge; 2*96485*V_Volume*Mass_Peroxide/dt];
    
    D_H2O2 = dCdT(H2O2_Concentration, H2O2_Dcoeffs, Voxel_Size); 
    H2O2_Concentration = H2O2_Concentration + D_H2O2*dt;

    D_Lactate = dCdT(Lactate_Concentration, Lactate_Dcoeffs, Voxel_Size); 
    Lactate_Concentration = Lactate_Concentration + D_Lactate*dt;
    
    if Snapshot_Counter >= Snapshot_Spread
        if Store_Data
            H2O2_Snapshot{end+1} = H2O2_Concentration;
            Lactate_Snapshot{end+1} = Lactate_Concentration;
        end
        
        if Write_Video
            [f1, Slice_Plot] = update_plot(f1, H2O2_Concentration, Volume, Slice_Plot, x, y, z,...
                      X_Slices, Y_Slices, Z_Slices, Aspect_Ratio, View_Angle, X_Limits, ...
                      Y_Limits, Z_Limits, Boundaries, Color_Limit, 1, Cross_Section, ... 
                      CS_Direction, CS_Position);
            
            set(0, 'currentfigure', f1);
            drawnow
            Out_Frame(end+1) = getframe(f1);

            [f2, Slice_Plot_Lactate] = update_plot(f2, Lactate_Concentration, Volume, Slice_Plot, x, y, z,...
            X_Slices, Y_Slices, Z_Slices, Aspect_Ratio, View_Angle, X_Limits, ...
            Y_Limits, Z_Limits, Boundaries, Color_Limit, 0, Cross_Section, ... 
            CS_Direction, CS_Position);

            set(0, 'currentfigure', f2);
            drawnow
            Out_Frame_Lactate(end+1) = getframe(f2);
        end

        Time_Points = [Time_Points; Time(i)];
        
        H2O2_Point_Concentrations = [H2O2_Point_Concentrations reshape(H2O2_Concentration(Membrane_X_Midpoint, Membrane_Y_Midpoint, :), [length(z), 1])];
        Lactate_Point_Concentrations = [Lactate_Point_Concentrations reshape(Lactate_Concentration(Membrane_X_Midpoint, Membrane_Y_Midpoint, :), [length(z), 1])];
        
        Snapshot_Counter = 0;
    end
    

    if CS_Counter >= CS_Spread
        if Cross_Section
            if strcmp(CS_Direction, 'x')
                [~, S_Index] = min(abs(x - CS_Position));
                C_Section =  reshape(H2O2_Concentration(S_Index, :, :), [length(y), length(z)]);
            elseif strcmp(CS_Direction, 'y')
                [~, S_Index] = min(abs(y - CS_Position));
                C_Section =  reshape(H2O2_Concentration(:, S_Index, :), [length(x), length(z)]);
            elseif strcmp(CS_Direction, 'z')
                [~, S_Index] = min(abs(z - CS_Position));
                C_Section =  reshape(H2O2_Concentration(:, :, S_Index), [length(x), length(y)]);            
            else
                disp('Invalid Slice Direction');
            end

            F_CS{end+1} = figure('Color', 'w');
            F_CS = plot_CS(F_CS, x, y, z, CS_Direction, C_Section, ...
                Aspect_Ratio, X_Limits, Y_Limits, Z_Limits, Boundaries,...
                Color_Limit, Plot_Scale);
        end
        
        CS_Counter = 0;
    end
    
    
    Snapshot_Counter = Snapshot_Counter + 1;
    CS_Counter = CS_Counter + 1;

    waitbar(i/length(Time), Waitbar, strcat(num2str(i), '/', num2str(length(Time))));

    if ~isempty(find(H2O2_Concentration > C_Threshold))
        disp('Max concentration exceeded:');
        disp('Stopping simulation')
        return;
    end
end
close(Waitbar);

if Save_Final
    save(strcat(H2O2_Output, '.mat'), 'H2O2_Concentration');
    save(strcat(Lactate_Output, '.mat'), 'Lactate_Concentration');
end

if Reduction
    figure; plot(Time, -1*Charge, 'LineWidth', Plot_Scale*2.5, 'Color', 'k'); 
    xlim([Time(3), Time(end)]);
    
    xlabel('Time (s)');
    ylabel('Current (A)');
    Format_Plot(gcf, Plot_Scale, '2D');
end


figure(); hold on;
jetcustom = jet(size(H2O2_Point_Concentrations, 1));
for i = 1:size(H2O2_Point_Concentrations, 1)
    plot3(Time_Points, z(i)*ones(size(H2O2_Point_Concentrations(i, :))), ...
          H2O2_Point_Concentrations(i, :), 'LineWidth', Plot_Scale*2.5, 'Color', jetcustom(i, :)); 
end
hold off; box on;
view([50, 10]);

xlabel('Time (s)');
ylabel('z position (mm)');
zlabel('H_{2}O_{2} concentration (M)');
Format_Plot(gcf, Plot_Scale, '3D');

if Write_Video
    warning('off');
    NV = VideoWriter(H2O2_Output, 'MPEG-4');
    NV.FrameRate = size(Out_Frame, 2)/Frame_Length;
    open(NV);
    writeVideo(NV, Out_Frame);
    close(NV);
    warning('on');

    warning('off');
    NV = VideoWriter(Lactate_Output, 'MPEG-4');
    NV.FrameRate = size(Out_Frame_Lactate, 2)/Frame_Length;
    open(NV);
    writeVideo(NV, Out_Frame_Lactate);
    close(NV);
    warning('on');
end

function [dsum] = dCdT(C, D, vxlsze)
    DX = (D(2:end, :, :).*D(1:end-1, :, :)).^(0.5); DX = DX;
    LHS = cat(1, zeros(1, size(C, 2), size(C, 3)), -1*DX.*(C(2:end, :, :)-C(1:end-1, :, :)));
    RHS = cat(1, -1*DX.*(C(1:end-1, :, :)-C(2:end, :, :)), zeros(1, size(C, 2), size(C, 3)));

    dx = (LHS+RHS);

    DY = (D(:, 2:end, :).*D(:, 1:end-1, :)).^(0.5); DY = DY;
    LHS = cat(2, zeros(size(C, 1), 1, size(C, 3)), -1*DY.*(C(:, 2:end, :)-C(:, 1:end-1, :)));
    RHS = cat(2, -1*DY.*(C(:, 1:end-1, :)-C(:, 2:end, :)), zeros(size(C, 1), 1, size(C, 3)));

    dy = (LHS+RHS);

    DZ = (D(:, :, 2:end).*D(:, :, 1:end-1)).^(0.5); DZ = DZ;
    LHS = cat(3, zeros(size(C, 1), size(C, 2), 1), -1*DZ.*(C(:, :, 2:end)-C(:, :, 1:end-1)));
    RHS = cat(3, -1*DZ.*(C(:, :, 1:end-1)-C(:, :, 2:end)), zeros(size(C, 1), size(C, 2), 1));

    dz = (LHS+RHS);
    dsum = (dx+dy+dz)/(vxlsze^2);
end

% define boundary conditions, tpe == 1 denotes real space boundary
% conditions, tpe == 0 dneotes first derivative boundary conditions
function [H2O2_Concentration, Lactate_Concentration, Mass_Peroxide] = Bdry_Conditions(H2O2_Concentration,...
    Lactate_Concentration,Cathode_Points, Anode_Points, Membrane_Points, ...
    Electrode_Height_Index, Membrane_Height_Index, H2O2_Bulk, Lactate_Bulk, Reduction)
    
    Mass_Peroxide = 0;
    % Set electrode peroxide concentration to 0
    if Reduction
        for i = 1:size(Cathode_Points, 1)
            Mass_Peroxide = Mass_Peroxide + H2O2_Concentration(Cathode_Points(i, 1), ...
                                                   Cathode_Points(i, 2),...
                                                   Cathode_Points(i, 3)); 
                                               
            H2O2_Concentration(Cathode_Points(i, 1), ...
                Cathode_Points(i, 2), Cathode_Points(i, 3)) = 0;
        end
    end

    Lactate_Concentration(1, :, :) = Lactate_Bulk;
    Lactate_Concentration(end, :, :) = Lactate_Bulk;
    Lactate_Concentration(:, 1, :) = Lactate_Bulk;
    Lactate_Concentration(:, end, :) = Lactate_Bulk;
    Lactate_Concentration(:, :, end) = Lactate_Bulk;

    H2O2_Concentration(1, :, :) = H2O2_Bulk;
    H2O2_Concentration(end, :, :) = H2O2_Bulk;
    H2O2_Concentration(:, 1, :) = H2O2_Bulk;
    H2O2_Concentration(:, end, :) = H2O2_Bulk;
    H2O2_Concentration(:, :, end) = H2O2_Bulk;
end

function [H2O2_Concentration, Lactate_Concentration] = Menten(H2O2_Concentration, ...
    Lactate_Concentration, Cathode_Points, Membrane_Points, ...
    Electrode_Height_Index, Membrane_Height_Index, dt, Menten_Coeff, ...
    Max_Velocity)
    
    Rate_Matrix = zeros(size(Lactate_Concentration));
        
    for i = 1:size(Membrane_Points, 1)
        for j = Electrode_Height_Index:Membrane_Height_Index
            Rate_Matrix(Membrane_Points(i, 1), Membrane_Points(i, 2), j) = ...
                Max_Velocity*Lactate_Concentration(Membrane_Points(i, 1),...
                Membrane_Points(i, 2), j)/(Menten_Coeff + ...
                Lactate_Concentration(Membrane_Points(i, 1), Membrane_Points(i, 2),...
                j));
        end
    end

    if ~any(Rate_Matrix)
        %disp('Zero Menten matrix');
    end

    dLactate = dt*Rate_Matrix.*Lactate_Concentration;
    dLactate(dLactate>Lactate_Concentration) = Lactate_Concentration(dLactate>Lactate_Concentration);

    Lactate_Concentration = Lactate_Concentration - dLactate;
    H2O2_Concentration = H2O2_Concentration + dLactate;

end

function [C] = Define_Dcoeffs(Mesh_List, x, y, z, C, Volume, val)    
    % Loop through all faces of Volume object
    for i = 1:size(Volume.face.vertex_indices, 1)
        
        % Find all x,y pairs below face at index i
        tface = Volume.face.vertex_indices{i}+1;
        tverts = [Volume.vertex.x(tface) Volume.vertex.y(tface) Volume.vertex.z(tface)];        
        in =  inpolygon(Mesh_List(:, 1), Mesh_List(:, 2), tverts(:, 1), tverts(:, 2));
        tpnts = [Mesh_List(in, :)];
        
        % Compute normal vector and plane equation
        N = cross((tverts(3, :) - tverts(2, :)), (tverts(1, :) - tverts(2, :)));
        d = N(1)*tverts(2, 1) + N(2)*tverts(2, 2) + N(3)*tverts(2, 3);
        
        % Compute plane height for each viable x,y pair
        for j = 1:size(tpnts, 1)
            if dot([0; 0; 1], N) > 1E-8
                h = (d-N(1)*tpnts(j, 1)-N(2)*tpnts(j, 2))/N(3);
            else
                h = max(tverts(:, 3));
            end
            
            xdex = find(x==tpnts(j, 1)); ydex = find(y==tpnts(j, 2));
            [~, zdex] = min(abs(z - h));
            
            C(xdex, ydex, 1:zdex) = val;
        end   
    end
end

function [f1, Slice_Plot] = update_plot(f1, Data_Matrix, Volume, Slice_Plot, x, y, z,...
    X_Slices, Y_Slices, Z_Slices, Aspect_Ratio, View_Angle, X_Limits, ...
    Y_Limits, Z_Limits, Boundaries, Color_Limit, Map_Option, Cross_Section, ... 
    CS_Direction, CS_Position)
    set(0, 'currentfigure', f1);
    clf(f1);

    Slice_Plot = slice(x, y, z, permute(Data_Matrix, [2, 1, 3]),...
                       X_Slices, Y_Slices, Z_Slices);

    colormap jet; 
    if ~isempty(Color_Limit); caxis(Color_Limit); end;

    set(Slice_Plot, 'EdgeColor', 'none', 'FaceColor', 'interp', 'FaceAlpha', 'interp'); 
    alpha('color');

    if Map_Option
        alphamap('rampup');
    else
        alphamap('rampdown');
    end


    view(View_Angle); ax = gca; %box on; ax.LineWidth = 3;

    set(get(gca, 'XAxis'), 'Visible', 'off');
    set(get(gca, 'YAxis'), 'Visible', 'off');
    set(get(gca, 'ZAxis'), 'Visible', 'off');
    %set(gca, 'Color', 'none');

    xticks([]); yticks([]); zticks([]); grid off;

    hold on;
    for i = 1:size(Volume.face.vertex_indices, 1)
        v_index = Volume.face.vertex_indices{i}+1;
        xgm = Volume.vertex.x(v_index);
        ygm = Volume.vertex.y(v_index);
        zgm = Volume.vertex.z(v_index);
        zgm(zgm<1E-6) = 0;
        if any(zgm, 'all');
            fill3(xgm, ygm, zgm, [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]); %'FaceAlpha', 1, 'LineStyle', 'none');
            fill3(xgm, ygm, -zgm, [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5]); %'FaceAlpha', 1, 'LineStyle', 'none');
        end
    end
    hold off;

    if ~isempty(X_Limits)
        xlim(X_Limits);
        Aspect_Ratio(1) = X_Limits(2) - X_Limits(1);
    else
        xlim([Boundaries(1, 1) Boundaries(1, 2)]);
    end

    if ~isempty(Y_Limits)
        ylim(Y_Limits);
        Aspect_Ratio(2) = Y_Limits(2) - Y_Limits(1);
    else
        ylim([Boundaries(2, 1) Boundaries(2, 2)]);
    end

    if ~isempty(Z_Limits)
        zlim(Z_Limits);
        Aspect_Ratio(3) = Z_Limits(2) - Z_Limits(1);
    else
        zlim([Boundaries(3, 1) Boundaries(3, 2)]);
    end
    pbaspect(Aspect_Ratio);

    if Cross_Section
        Line_Vector = [];
        if strcmp(CS_Direction, 'x')
            D1 = ylim; D2 = zlim;
            Line_Vector = [CS_Position D1(1) D2(1);
                           CS_Position D1(1) D2(2);
                           CS_Position D1(2) D2(2);
                           CS_Position D1(2) D2(1);
                           CS_Position D1(1) D2(1)];
        
        elseif strcmp(CS_Direction, 'y')
            D1 = xlim; D2 = zlim;
            Line_Vector = [D1(1) CS_Position D2(1);
                           D1(1) CS_Position D2(2);
                           D1(2) CS_Position D2(2);
                           D1(2) CS_Position D2(1);
                           D1(1) CS_Position D2(1)];
        
        elseif strcmp(CS_Direction, 'z')
            D1 = xlim; D2 = ylim;
            Line_Vector = [D1(1) D2(1) CS_Position;
                           D1(1) D2(2) CS_Position;
                           D1(2) D2(2) CS_Position;
                           D1(2) D2(1) CS_Position;
                           D1(1) D2(1) CS_Position];
        end
        hold on;
        plot3(Line_Vector(:, 1), Line_Vector(:, 2), Line_Vector(:, 3), ...
            'LineWidth', 1.5, 'LineStyle', '-.', 'Color', 'k');
        hold off;
    end

    drawnow;
end

function [F_CS] = plot_CS(F_CS, x, y, z, CS_Direction, C_Section, Aspect_Ratio,...
                          X_Limits, Y_Limits, Z_Limits, Boundaries,...
                          Color_Limit, Plot_Scale)
    
    set(0, 'currentfigure', F_CS{end}); set(gcf, 'Units', 'inches');
    set(gcf, 'Position', [0.25 1 Plot_Scale*5.36 Plot_Scale*4.1025]);
    
    if strcmp(CS_Direction, 'x')
        surf(z, y, C_Section); ax = gca; view(-90, 90);
        pbaspect([z(end)-z(1) y(end)-y(1) 1]); 
        if ~isempty(Z_Limits)
            xlim(Z_Limits);
            ax.PlotBoxAspectRatio(1) = Z_Limits(2) - Z_Limits(1);
        end
        
        if ~isempty(Y_Limits)
            ylim(Y_Limits);
            ax.PlotBoxAspectRatio(2) = Y_Limits(2) - Y_Limits(1);
        end        
       
        
    elseif strcmp(CS_Direction, 'y')
        surf(z, x, C_Section); ax = gca; view(-90, 90);
        pbaspect([z(end)-z(1) x(end)-x(1) 1]); 
        if ~isempty(Z_Limits)
            xlim(Z_Limits);
            ax.PlotBoxAspectRatio(1) = Z_Limits(2) - Z_Limits(1);
        end
        
        if ~isempty(X_Limits)
            ylim(X_Limits);
            ax.PlotBoxAspectRatio(2) = X_Limits(2) - X_Limits(1);
        end        

    elseif strcmp(CS_Direction, 'z')
        surf(y, x, C_Section); ax = gca; view(-90, 90);
        pbaspect([y(end)-y(1) x(end)-x(1) 1]); 
        if ~isempty(Y_Limits)
            xlim(Y_Limits);
            ax.PlotBoxAspectRatio(1) = Y_Limits(2) - Y_Limits(1);
        end
        
        if ~isempty(X_Limits)
            ylim(X_Limits);
            ax.PlotBoxAspectRatio(2) = X_Limits(2) - X_Limits(1);
        end  
    end
    
    colormap jet; shading interp;
    box on; ax.LineWidth = 2.5;
    xticks([]); yticks([]);
    set(gca, 'Position', [0.1786,0.1663,0.6819,0.7179]);
    set(gca, 'FontSize', Plot_Scale*15);
    set(gca, 'XColor', [0, 0, 0]);
    set(gca, 'YColor', [0, 0, 0]);
    
    c = colorbar; if ~isempty(Color_Limit); caxis(Color_Limit); end;
    c.FontName = 'Arial';
    c.FontSize = Plot_Scale*15.0;
    c.Label.String= 'H_{2}O_{2} concentration (M)';
    c.FontWeight = 'bold';
    c.LineWidth = Plot_Scale*2.5;
    c.Color = [0, 0, 0];
    c.Position = [0.82,0.1663,0.035,0.7179];
    
end

function Format_Plot(f, Plot_Scale, Dimension)
    set(0, 'currentfigure', f);
    
    set(f, 'Color', 'w');
    set(f, 'Units', 'inches');
    set(f, 'Position', [0.25 1 Plot_Scale*5.36 Plot_Scale*4.1025]);
    set(gca, 'Color', 'none');
    set(gca, 'Position', [0.1786,0.1663,0.6819,0.7179]);
    set(gca, 'LineWidth', Plot_Scale*2.5);
    set(gca, 'XColor', [0, 0, 0]);
    set(gca, 'YColor', [0, 0, 0]);
    set(gca, 'FontName', 'Arial');
    set(gca, 'FontSize', Plot_Scale*15);
    set(gca, 'FontWeight', 'bold');
   
    if strcmp(Dimension, '2D')
        box off;
        b = xlim; xline(b(2), 'LineWidth', Plot_Scale*2.5, 'Color', 'k', 'Alpha', 1);
        b = ylim; yline(b(2), 'LineWidth', Plot_Scale*2.5, 'Color', 'k', 'Alpha', 1);
    end
  
end
