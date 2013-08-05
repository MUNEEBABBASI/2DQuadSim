% 7/29/13
% animateQuadLoad.m
% animates quadrotor motion in a gui

function varargout = animateQuadLoad(varargin)
% ANIMATEQUADLOAD MATLAB code for animateQuadLoad.fig
%      ANIMATEQUADLOAD, by itself, creates a new ANIMATEQUADLOAD or raises the existing
%      singleton*.
%
%      H = ANIMATEQUADLOAD returns the handle to a new ANIMATEQUADLOAD or the handle to
%      the existing singleton*.
%
%      ANIMATEQUADLOAD('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANIMATEQUADLOAD.M with the given input arguments.
%
%      ANIMATEQUADLOAD('Property','Value',...) creates a new ANIMATEQUADLOAD or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before animateQuadLoad_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to animateQuadLoad_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help animateQuadLoad

% Last Modified by GUIDE v2.5 05-Aug-2013 17:04:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @animateQuadLoad_OpeningFcn, ...
                   'gui_OutputFcn',  @animateQuadLoad_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before animateQuadLoad is made visible.
function animateQuadLoad_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to animateQuadLoad (see VARARGIN)

% Choose default command line output for animateQuadLoad
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes animateQuadLoad wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% create global variable containing all the relevant information
global s

s.currentTimeIndex = 1;
s.simSpeed = 3;
s.time = evalin('base', 'tout');
s.quadLength = 0.5;

s.modes = evalin('base', 'modeout');
s.xQ = evalin('base','quadx');
s.vQ = evalin('base','quadv');
s.phiQ = evalin('base', 'quadphi(:, 1)');
s.phidotQ = evalin('base', 'quadphi(:, 2)');
s.xL = evalin('base', 'xout(:, 1:2)');
s.vL = evalin('base', 'xout(:, 3:4)');
s.phiL = evalin('base','loadphi(:, 1)');
s.phidotL = evalin('base', 'loadphi(:, 2)');
s.u = evalin('base', 'uout');
s.des = evalin('base', 'desout');
s.xTraj = evalin('base', 'xTraj');
s.dxTraj = evalin('base', 'dxTraj');
s.d2xTraj = evalin('base', 'd2xTraj');
s.d3xTraj = evalin('base', 'd3xTraj');
s.d4xTraj = evalin('base', 'd4xTraj');
s.d5xTraj = evalin('base', 'd5xTraj');
s.d6xTraj = evalin('base', 'd6xTraj');

% set slider properties
sliderMin = 1;
sliderMax = 5;
sliderStep = [1, 1]/(sliderMax-sliderMin); % major and minor steps of 1

set(handles.slider1, 'Min', sliderMin);
set(handles.slider1, 'Max', sliderMax);
set(handles.slider1, 'SliderStep', sliderStep);
set(handles.slider1, 'Value', 3); 

% plot outputs in gui
axes(handles.axes2)
hold on
box on
plot(s.time,s.xL(:, 1),'r');
plot(s.time,s.xL(:, 2),'b');
plot(s.time,s.xTraj(:, 1),'r--');
plot(s.time,s.xTraj(:, 2),'b--');
%s.limits2 = get(handles.axes2,'YLim');
%s.line2 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits2);
legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
ylabel('load pos (m)');

axes(handles.axes3)
hold on
box on
plot(s.time,s.vL(:, 1),'r');
plot(s.time,s.vL(:, 2),'b');
plot(s.time,s.dxTraj(:, 1),'r--');
plot(s.time,s.dxTraj(:, 2),'b--');
%s.limits3 = get(handles.axes3, 'YLim');
%s.line3 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits3);
legend('actual ydot', 'actual zdot', 'desired ydot', 'desired zdot', 'Location', 'SouthEastOutside');
ylabel('load vel (m/s)');

axes(handles.axes4)
hold on
box on
plot(s.time,s.phiL.*180./pi,'r');
plot(s.time,s.phidotL.*180./pi,'b');
plot(s.time,s.des(:, 4).*180./pi,'r--');
%s.limits4 = get(handles.axes4, 'YLim');
%s.line4 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits4);
legend('phiL', 'phidotL', 'phiL des', 'Location', 'SouthEastOutside');
ylabel('load angle (deg, s)');

axes(handles.axes5)
hold on
box on
plot(s.time,s.xQ(:, 1),'r');
plot(s.time,s.xQ(:, 2),'b');
plot(s.time,s.des(:, 1),'r--');
plot(s.time,s.des(:, 2),'b--');
%s.limits5 = get(handles.axes5, 'YLim');
%s.line5 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits5);
legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
ylabel('quad pos (m)');

axes(handles.axes6)
hold on
box on
plot(s.time,s.vQ(:, 1),'r');
plot(s.time,s.vQ(:, 2),'b');
plot(s.time,s.des(:, 3),'r--');
plot(s.time,s.des(:, 4),'b--');
%s.limits6 = get(handles.axes6, 'YLim');
%s.line6 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits6);
legend('actual ydot', 'actual zdot', 'desired ydot', 'desired zdot', 'Location', 'SouthEastOutside');
ylabel('quad vel (m/s)');

axes(handles.axes7)
hold on
box on
plot(s.time,s.phiQ.*180./pi,'r');
plot(s.time,s.phidotQ.*180./pi,'b');
plot(s.time,s.des(:, 3).*180./pi,'r--');
%s.limits7 = get(handles.axes7, 'YLim');
%s.line7 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits7);
legend('phiQ', 'phidotQ', 'phiQ des', 'Location', 'SouthEastOutside');
ylabel('quad angle (deg, s)');


% plot inputs
axes(handles.axes8)
hold on
box on
plot(s.time,s.u(:, 1),'r');
%s.limits8 = get(handles.axes8, 'YLim');
%s.line8 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits8);
ylabel('f');

axes(handles.axes9)
hold on
box on
plot(s.time,s.u(:, 2),'r');
%s.limits9 = get(handles.axes9, 'YLim');
%s.line9 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits9);
ylabel('M');



% plot tension over time
figure()
T = zeros(length(s.d2xTraj(:, 1)), 1);
for i = 1:length(s.d2xTraj(:, 1))
    T(i, 1) = s.mL.* norm ( [s.d2xTraj(i, 1); s.d2xTraj(i, 2)] + [0; s.g]);
end

axes(handles.axes10)
hold on
box on
plot(s.time,T,'r');
%s.limits10 = get(handles.axes9, 'YLim');
%s.line10 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits10);
ylabel('tension');

linkaxes([handles.axes2, handles.axes3, handles.axes4, handles.axes5, handles.axes6, handles.axes7, handles.axes8, handles.axes9, handles.axes10], 'x');



%%%% 
% plot in 2D
% draw the quad rotor and load

global traj

    
axes(handles.axes1)
t = s.currentTimeIndex;
plot(s.xL(t, 1), s.xL(t, 2), 'ro', 'Markersize', 10, 'MarkerFaceColor', 'r');
hold on
grid on
box on
line([s.xQ(t, 1)-s.quadLength/2*cos(s.phiQ(t, 1)) s.xQ(t, 1)+s.quadLength/2*cos(s.phiQ(t, 1))], ...
    [s.xQ(t, 2)-s.quadLength/2*sin(s.phiQ(t, 1)) s.xQ(t, 2)+s.quadLength/2*sin(s.phiQ(t, 1))], 'Color', 'k');
%plot(s.xQ(t, 1), s.xQ(t, 2), 'k+', 'Markersize', 12);
plot(s.xTraj(:, 1), s.xTraj(:, 2), 'r--');
line([s.xQ(t, 1) s.xL(t, 1)], [s.xQ(t, 2) s.xL(t, 2)], 'Color', 'k', 'LineStyle', '--');
s.limits = [min([s.xQ(:, 1); s.xQ(:, 2); s.xL(:, 1); s.xL(:, 2)])-0.5 ...
    max([s.xQ(:, 1); s.xQ(:, 2); s.xL(:, 1); s.xL(:, 2)])+0.5];

if (length(traj.tDes) > 0),
plot(traj.posDes(1, :, 1), traj.posDes(1, :, 2), '^');
end
    
%legend('load pos', 'desired load pos', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
zlabel('z (m)');
ylabel('y (m)');
xlabel('x (m)');
set(gca, 'XLim', s.limits, 'YLim', s.limits);
%set(gca,'xlim',[-1 1], 'ylim',ylimits, 'zlim', ylimits);


% %%%% 
% % plot in 3D
% % draw the quad rotor and load
% axes(handles.axes1)
% t = s.currentTimeIndex;
% plot3(0, s.xL(t, 1), s.xL(t, 2), 'ro', 'Markersize', 10, 'MarkerFaceColor', 'r');
% hold on
% grid on
% plot3(0, s.xTraj(t, 1),s.xTraj(t, 2),'r--');
% plot3(0, s.xQ(t, 1), s.xQ(t, 2), 'k+', 'Markersize', 12);
% line([0 0], [s.xQ(t, 1) s.xL(t, 1)], [s.xQ(t, 2) s.xL(t, 2)], 'Color', 'k', 'LineStyle', '--');
% %legend('load pos', 'desired load pos', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
% zlabel('z (m)');
% ylabel('y (m)');
% xlabel('x (m)');
% set(gca, 'YLim', s.limits, 'ZLim', s.limits)
% %set(gca,'xlim',[-1 1], 'ylim',ylimits, 'zlim', ylimits);






% --- Outputs from this function are returned to the command line.
function varargout = animateQuadLoad_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1

global s

%gets the selected option
switch get(handles.popupmenu1,'Value')
    case 1
        axes(handles.axes2)
        hold off
        plot(s.time,s.xL(:, 1),'r');
        hold on
        box on
        plot(s.time,s.xL(:, 2),'b');
        plot(s.time,s.xTraj(:, 1),'r--');
        plot(s.time,s.xTraj(:, 2),'b--');
        %limitsTemp = get(handles.axes2, 'YLim');
        %set(s.line2, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
        ylabel('load pos (m)');
    case 2
        axes(handles.axes2)
        hold off
        plot(s.time,s.xL(:, 1) - s.xTraj(:, 1),'r');
        hold on
        box on
        plot(s.time,s.xL(:, 2) - s.xTraj(:, 2),'b');
        %limitsTemp = get(handles.axes7, 'YLim');
        %set(s.line2, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('y error', 'z error', 'Location', 'SouthEastOutside');
        ylabel('pos errors (m)')
    otherwise
end


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2

global s

%gets the selected option
switch get(handles.popupmenu2,'Value')
    case 1
        axes(handles.axes3)
        hold off
        plot(s.time,s.vL(:, 1),'r');
        hold on
        box on
        plot(s.time,s.vL(:, 2),'b');
        plot(s.time,s.dxTraj(:, 1),'r--');
        plot(s.time,s.dxTraj(:, 2),'b--');
        legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
        %limitsTemp = get(handles.axes7, 'YLim');  
        %set(s.line3, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        ylabel('load pos (m)');
    case 2
        axes(handles.axes3)
        hold off
        plot(s.time,s.vL(:, 1) - s.dxTraj(:, 1),'r');
        hold on
        box on
        plot(s.time,s.vL(:, 2) - s.dxTraj(:, 2),'b');
        legend('y error', 'z error', 'Location', 'SouthEastOutside');
        %limitsTemp = get(handles.axes3, 'YLim');
        %set(s.line4, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        ylabel('pos errors (m)')
    otherwise
end

% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3

global s

switch get(handles.popupmenu3,'Value')
    case 1     
        axes(handles.axes4)
        hold off
        plot(s.time,s.phiL.*180./pi,'r');
        hold on
        box on
        plot(s.time,s.phidotL.*180./pi,'b');
        plot(s.time,s.des(:, 6).*180./pi,'r--');
        %limitsTemp = get(handles.axes4, 'YLim');
        %set(s.line4, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('phiL', 'phidotL', 'phiL des', 'Location', 'SouthEastOutside');
        ylabel('load angle (deg, s)');
    case 2       
        axes(handles.axes4)
        hold off
        plot(s.time,(s.phiL-s.des(:, 6)).*180./pi,'r');
        hold on
        box on
        %limitsTemp = get(handles.axes4, 'YLim');
        %set(s.line4, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('phiL error', 'Location', 'SouthEastOutside');
        ylabel('load angle err (deg)');
    otherwise
end


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4

global s

switch get(handles.popupmenu4,'Value')
    case 1
        axes(handles.axes5)
        hold off
        plot(s.time,s.xQ(:, 1),'r');
        hold on
        box on
        plot(s.time,s.xQ(:, 2),'b');
        plot(s.time,s.des(:, 1),'r--');
        plot(s.time,s.des(:, 2),'b--');
        %limitsTemp = get(handles.axes5, 'YLim');
        %set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
        ylabel('quad pos (m)');

    case 2
        axes(handles.axes5)
        hold off
        plot(s.time,s.xQ(:, 1) - s.des(:, 1),'r');
        hold on
        box on
        plot(s.time,s.xQ(:, 2) - s.des(:, 2),'b');
        %limitsTemp = get(handles.axes5, 'YLim');
       % set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
       % ylim(limitsTemp);
        legend('y error', 'z error', 'Location', 'SouthEastOutside');
        ylabel('pos errors (m)')
    otherwise
end

% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% % --- Executes on selection change in popupmenu5.
% function popupmenu5_Callback(hObject, eventdata, handles)
% % hObject    handle to popupmenu5 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
% %        contents{get(hObject,'Value')} returns selected item from popupmenu5
% 
% global s
% 
% switch get(handles.popupmenu5,'Value')
%     case 1
%         axes(handles.axes5)
%         hold off
%         plot(s.time,s.xQ(:, 1),'r');
%         hold on
%         plot(s.time,s.xQ(:, 2),'b');
%         plot(s.time,s.des(:, 1),'r--');
%         plot(s.time,s.des(:, 2),'b--');
%         limitsTemp = get(handles.axes5, 'YLim');
%         %set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%         ylim(limitsTemp);
%         legend('actual y', 'actual z', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
%         ylabel('quad pos (m)');
% 
%     case 2
%         axes(handles.axes5)
%         hold off
%         plot(s.time,s.xQ(:, 1) - s.des(:, 1),'r');
%         hold on
%         plot(s.time,s.xQ(:, 2) - s.des(:, 2),'b');
%         limitsTemp = get(handles.axes5, 'YLim');
%        % set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%         ylim(limitsTemp);
%         legend('y error', 'z error', 'Location', 'SouthEastOutside');
%         ylabel('pos errors (m)')
%     otherwise
% end
% 
% % --- Executes during object creation, after setting all properties.
% function popupmenu5_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to popupmenu5 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: popupmenu controls usually have a white background on Windows.
% %       See ISPC and COMPUTER.
% if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor','white');
% end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6

global s

switch get(handles.popupmenu6,'Value')
    case 1     
        axes(handles.axes7)
        hold off
        plot(s.time,s.phiQ.*180./pi,'r');
        hold on
        box on
        plot(s.time,s.phidotQ.*180./pi,'b');
        plot(s.time,s.des(:, 5).*180./pi,'r--');
        %limitsTemp = get(handles.axes7, 'YLim');
        %set(s.line7, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('phiQ', 'phidotQ', 'phiQ des', 'Location', 'SouthEastOutside');
        ylabel('quad angle (deg, s)');
    case 2       
        axes(handles.axes7)
        hold off
        plot(s.time,(s.phiQ-s.des(:, 5)).*180./pi,'r');
        hold on
        box on
       % limitsTemp = get(handles.axes7, 'YLim');
       % set(s.line7, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
       % ylim(limitsTemp);
        legend('phiQ error', 'Location', 'SouthEastOutside');
        ylabel('quad angle err (deg)');
    otherwise
end


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7

global s

switch get(handles.popupmenu7,'Value')
    case 1
        axes(handles.axes6)
        hold off
        plot(s.time,s.vQ(:, 1),'r');
        hold on
        box on
        plot(s.time,s.vQ(:, 2),'b');
        plot(s.time,s.des(:, 3),'r--');
        plot(s.time,s.des(:, 4),'b--');
        %limitsTemp = get(handles.axes6, 'YLim');
        %set(s.line6, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual ydot', 'actual zdot', 'desired ydot', 'desired zdot', 'Location', 'SouthEastOutside');
        ylabel('quad vel (m/s)');

    case 2
        axes(handles.axes6)
        hold off
        plot(s.time,s.vQ(:, 1) - s.des(:, 3),'r');
        hold on
        box on
        plot(s.time,s.vQ(:, 2) - s.des(:, 4),'b');
        %limitsTemp = get(handles.axes6, 'YLim');
        %set(s.line6, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('ydot error', 'zdot error', 'Location', 'SouthEastOutside');
        ylabel('vel errors (m/s)')
    otherwise
end


% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global traj
global s
stop = 0;

if (s.currentTimeIndex >= length(s.time) && stop == 0),
    s.currentTimeIndex = 1;
end
      
while (get(hObject,'Value') && stop==0)
    get(hObject,'Value');
    
    try
        
      % draw the quad rotor and load
      t = s.currentTimeIndex+1;
      %numWaypoints = length(traj.posT);

      axes(handles.axes1)
      
      %%%
      % 2D plot
      hold off
      plot(s.xL(t, 1), s.xL(t, 2), 'ro', 'Markersize', 10, 'MarkerFaceColor', 'r');
      hold on
      grid on
      box on
      %plot(s.xQ(t, 1), s.xQ(t, 2), 'k+', 'Markersize', 12);
      line([s.xQ(t, 1)-s.quadLength/2*cos(s.phiQ(t, 1)) s.xQ(t, 1)+s.quadLength/2*cos(s.phiQ(t, 1))], ...
            [s.xQ(t, 2)-s.quadLength/2*sin(s.phiQ(t, 1)) s.xQ(t, 2)+s.quadLength/2*sin(s.phiQ(t, 1))], 'Color', 'k');
      line([s.xQ(t, 1) s.xL(t, 1)], [s.xQ(t, 2) s.xL(t, 2)], 'Color', 'k', 'LineStyle', '--');
      plot(s.xTraj(:, 1), s.xTraj(:, 2), 'r--');
      
      if (length(traj.tDes) > 0),
      plot(traj.posDes(1, :, 1), traj.posDes(1, :, 2), '^');
      end

      %plot past positions
      plot(s.xL(1:t, 1), s.xL(1:t, 2), 'r');
      plot(s.xQ(1:t, 1), s.xQ(1:t, 2), 'k');      
      set(gca, 'XLim', s.limits, 'YLim', s.limits);
      %set(gca, 'XLim', [-0.5 3.5], 'YLim', [-0.5 3.5]);
      zlabel('z (m)');
      ylabel('y (m)');
      xlabel('x (m)');
      pause(1/(10^(s.simSpeed*5)));     
      
      
      
%       %%%
%       % 3D plot
%       %plot current position
%       hold off
%       plot3(0, s.xL(t, 1), s.xL(t, 2), 'ro', 'Markersize', 10, 'MarkerFaceColor', 'r');
%       hold on
%       grid on
%       plot3(0, s.xTraj(t, 1),s.xTraj(t, 2),'r--');
%       plot3(0, s.xQ(t, 1), s.xQ(t, 2), 'k+', 'Markersize', 12);
%       line([0 0], [s.xQ(t, 1) s.xL(t, 1)], [s.xQ(t, 2) s.xL(t, 2)], 'Color', 'k', 'LineStyle', '--');
%       
%       %plot past positions
%       plot3(zeros(length(s.time(1:t)), 1), s.xL(1:t, 1), s.xL(1:t, 2), 'r');
%       plot3(zeros(length(s.time), 1), s.xTraj(:, 1), s.xTraj(:, 2), 'r--');
%       plot3(zeros(length(s.time(1:t)), 1), s.xQ(1:t, 1), s.xQ(1:t, 2), 'k');      
%       set(gca, 'YLim', s.limits, 'ZLim', s.limits);
%       zlabel('z (m)');
%       ylabel('y (m)');
%       xlabel('x (m)');
%       pause(1/(10^s.simSpeed*3))
      
      s.currentTimeIndex = t;
      set(handles.text2,'String',num2str(s.time(t)));
      set(handles.text1,'String',['Mode ' num2str(s.modes(t))]);
%       set(s.line2, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%       set(s.line3, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%       set(s.line4, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%       set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%       set(s.line6, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
%       set(s.line7, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
      guidata(hObject, handles);
    catch
        stop = 1;
    end
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global s

s.simSpeed = get(hObject, 'Value');


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function axes7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes7


% --- Executes during object creation, after setting all properties.
function axes10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes10
