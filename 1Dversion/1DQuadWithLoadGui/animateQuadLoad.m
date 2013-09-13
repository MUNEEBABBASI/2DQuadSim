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
s.xL = evalin('base', 'xout(:, 1)');
s.vL = evalin('base', 'xout(:, 2)');
s.u = evalin('base', 'uout');
s.xTraj = evalin('base', 'xTraj');
s.dxTraj = evalin('base', 'dxTraj');
s.d2xTraj = evalin('base', 'd2xTraj');
s.xTrajQ = evalin('base', 'xTrajQ');
s.dxTrajQ = evalin('base', 'dxTrajQ');
s.d2xTrajQ = evalin('base', 'd2xTrajQ');
s.T = evalin('base', 'T');


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
cla
hold on
box on
grid on
plot(s.time,s.xL(:, 1),'r');
plot(s.time,s.xTraj(:, 1),'r--');
%s.limits2 = get(handles.axes2,'YLim');
%s.line2 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits2);
legend('actual x', 'desired x', 'Location', 'SouthEastOutside');
ylabel('load pos (m)');

axes(handles.axes3)
cla
hold on
box on
grid on
plot(s.time,s.vL(:, 1),'r');
plot(s.time,s.dxTraj(:, 1),'r--');
%s.limits3 = get(handles.axes3, 'YLim');
%s.line3 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits3);
legend('actual xdot', 'desired xdot', 'Location', 'SouthEastOutside');
ylabel('load vel (m/s)');



axes(handles.axes5)
cla
hold on
box on
grid on 
plot(s.time,s.xQ(:, 1),'r');
plot(s.time,s.xTrajQ(:, 1),'r--');
%s.limits5 = get(handles.axes5, 'YLim');
%s.line5 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits5);
legend('actual x', 'desired x', 'Location', 'SouthEastOutside');
ylabel('quad pos (m)');

axes(handles.axes6)
cla
hold on
box on
grid on
plot(s.time,s.vQ(:, 1),'r');
plot(s.time,s.dxTrajQ(:, 1),'r--');
%s.limits6 = get(handles.axes6, 'YLim');
%s.line6 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits6);
legend('actual xdot', 'desired xdot', 'Location', 'SouthEastOutside');
ylabel('quad vel (m/s)');


% plot inputs
axes(handles.axes8)
cla
hold on
box on
grid on
plot(s.time,s.u(:, 1),'r');
%s.limits8 = get(handles.axes8, 'YLim');
%s.line8 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits8);
ylabel('f');




% plot tension over time
figure()
axes(handles.axes10)
cla
hold on
box on
grid on
plot(s.time(1:length(s.time)-1),s.T,'r');
%s.limits10 = get(handles.axes9, 'YLim');
%s.line10 = line([s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)],[-100,100]);
%ylim(s.limits10);
ylabel('tension');

linkaxes([handles.axes2, handles.axes3, handles.axes5, handles.axes6, handles.axes8, handles.axes10], 'x');



%%%% 
% plot in 2D
% draw the quad rotor and load

global traj

    
axes(handles.axes1)
t = s.currentTimeIndex;
plot(0, s.xL(t, 1), 'ro', 'Markersize', 12, 'MarkerFaceColor', 'r');
hold on
grid on
box on
line([-s.quadLength s.quadLength], ...
    [s.xQ(t, 1) s.xQ(t, 1)], ...
    'Color', 'k', 'LineWidth', 1.2);
plot(0, s.xTraj(:, 1), 'r--');

if (s.modes(t, 1) == 2) 
    color = 'b';
else
    color = 'k';
end

line([0 0], [s.xQ(t, 1) s.xL(t, 1)], 'Color', color, 'LineStyle', '--', 'LineWidth', 1.5);
s.limits = [min([s.xQ(:, 1); s.xL(:, 1); 0])-0.5 ...
    max([s.xQ(:, 1); s.xL(:, 1); 0])+0.5];

if (length(traj.tDes) > 0),
    plot(0, traj.posDes(1, :, 1), 'k^');
end
    
%legend('load pos', 'desired load pos', 'desired y', 'desired z', 'Location', 'SouthEastOutside');
ylabel('z (m)');
xlabel('y (m)');
set(gca, 'XLim', [-(s.limits(1, 2)-s.limits(1, 1))/2 (s.limits(1, 2)-s.limits(1, 1))/2], 'YLim', s.limits);







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
        grid on
        box on
        plot(s.time,s.xTraj(:, 1),'r--');
        %limitsTemp = get(handles.axes2, 'YLim');
        %set(s.line2, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual x', 'desired x', 'Location', 'SouthEastOutside');
        ylabel('load pos (m)');
    case 2
        axes(handles.axes2)
        hold off
        plot(s.time,s.xL(:, 1) - s.xTraj(:, 1),'r');
        hold on
        box on
        grid on
        %limitsTemp = get(handles.axes7, 'YLim');
        %set(s.line2, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('x error', 'Location', 'SouthEastOutside');
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
        grid on
        plot(s.time,s.dxTraj(:, 1),'r--');
        legend('actual x', 'desired x', 'Location', 'SouthEastOutside');
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
        grid on
        legend('x error', 'Location', 'SouthEastOutside');
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
        grid on
        plot(s.time,s.xTrajQ(:, 1),'r--');
        %limitsTemp = get(handles.axes5, 'YLim');
        %set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual x', 'desired x', 'Location', 'SouthEastOutside');
        ylabel('quad pos (m)');

    case 2
        axes(handles.axes5)
        hold off
        plot(s.time,s.xQ(:, 1) - (s.xTrajQ(:, 1)),'r');
        hold on
        box on
        grid on
        %limitsTemp = get(handles.axes5, 'YLim');
       % set(s.line5, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
       % ylim(limitsTemp);
        legend('x error', 'Location', 'SouthEastOutside');
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
        grid on
        plot(s.time,s.dxTrajQ(:, 1),'r--');
        %limitsTemp = get(handles.axes6, 'YLim');
        %set(s.line6, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('actual xdot', 'desired xdot', 'Location', 'SouthEastOutside');
        ylabel('quad vel (m/s)');

    case 2
        axes(handles.axes6)
        hold off
        plot(s.time,s.vQ(:, 1) - s.dxTrajQ(:, 1),'r');
        hold on
        box on
        grid on
        %limitsTemp = get(handles.axes6, 'YLim');
        %set(s.line6, 'XData', [s.time(s.currentTimeIndex),s.time(s.currentTimeIndex)]);
        %ylim(limitsTemp);
        legend('xdot error', 'Location', 'SouthEastOutside');
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
      plot(0, s.xL(t, 1), 'ro', 'Markersize', 12, 'MarkerFaceColor', 'r');
      hold on
      grid on
      box on
      line([-s.quadLength s.quadLength], ...
        [s.xQ(t, 1) s.xQ(t, 1)], ...
        'Color', 'k', 'LineWidth', 1.2);
      plot(0, s.xTraj(:, 1), 'r--');
      
      if (s.modes(t, 1) == 2)
          color = 'b';
      else
          color = 'k';
      end
      line([0 0], [s.xQ(t, 1) s.xL(t, 1)], 'Color', color, 'LineStyle', '--', 'LineWidth', 1.5);

      
      if (length(traj.tDes) > 0),
        plot(0, traj.posDes(1, :, 1), 'k^');
      end

      %plot past positions
      plot(0, s.xL(1:t, 1), 'r');
      plot(0, s.xQ(1:t, 1), 'k');      
      set(gca, 'XLim', [-(s.limits(1, 2)-s.limits(1, 1))/2 (s.limits(1, 2)-s.limits(1, 1))/2], 'YLim', s.limits);
      ylabel('z (m)');
      xlabel('y (m)');

      pause(1/(10^(s.simSpeed-1)));     
      

      
      s.currentTimeIndex = t;
      set(handles.text2,'String',num2str(s.time(t)));
      set(handles.text1,'String',['Mode ' num2str(s.modes(t))]);

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
function axes10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes10
