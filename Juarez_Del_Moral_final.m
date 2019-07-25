% Hayim Enrique Juarez Del Moral
% Program Description: Umbrella arm that expands and retracts from a link
        % input. It will tilt beforhand to adjust angle of the umbrella from a rack
        % and pinion set up.
% INPUTS: for loops of crank-slider and umbrella arm movememnt. Umbrella 
        % stem and crank will be inputs of the for loops.  
% OUTPUTS: Plot of umbrella and crank movement. Plot of inputs vs outputs 
% DATE:12/05/2018
% VERSION: 1.0 
%% Newtons Method to solve for awning unkowns
% ususual commands to prep the program
clc
clear

%conversion items
rad2deg = 180/pi;
deg2rad = pi/180;

RS1 = 5; % lengths of the fourbar
RS2 = 6; 
RS3 = 2;
RS4 = 3.5;
RS3EXT = 4;  %length of the extended arm

Stheta1 = 90*deg2rad;   %angle of fourbar ground
Stheta4 = linspace(-45,15)*deg2rad;   %fourbar input 
% Tolerance
tolerance = 10^(-3);

% Error
error = 2*tolerance;

% Initial Guess for theta 2 and theta 3
guess = [320 ; 70]*deg2rad;

theta2 = guess(1);   %assigning guesses to variable for better clarity.
theta3 = guess(2);

%array to hold values
output = zeros(length(Stheta4),2);

%% Application of Newton's Method to determine the values of theta2 and
% theta3

for i = 1:length(Stheta4)
    
    while error > tolerance

    % vector loop equations for the electric rope shovel with the initial
    % guess
     FUN = [(RS1*cos(Stheta1))+(RS2*cos(theta2))-(RS3*cos(theta3))-...
            (RS4*cos(Stheta4(i)))
            (RS1*sin(Stheta1))+(RS2*sin(theta2))-(RS3*sin(theta3))-...
            (RS4*sin(Stheta4(i)))];
           
    % Definition of the Jacobian matrix for the electric rope shovel
    % (derived from the vector loop equation for the electric rope shovel)
     J1(1,1) = (-RS2*sin(theta2));
     J1(1,2) = (RS3*sin(theta3));

     J1(2,1) = (RS2*cos(theta2));
     J1(2,2) = (-RS3*cos(theta3));

    % Calculation of the change in the guess
     dtheta = -inv(J1)*FUN;

    % Addition of the change in the guess to the previous guess
     guess = guess + dtheta;
     theta2 = guess(1);
     theta3 = guess(2);

    % update f1, f2, and f3
     FUN = [(RS1*cos(Stheta1))+(RS2*cos(theta2))-(RS3*cos(theta3))-...
            (RS4*cos(Stheta4(i)))
            (RS1*sin(Stheta1))+(RS2*sin(theta2))-(RS3*sin(theta3))-...
            (RS4*sin(Stheta4(i)))];

    % Calculation of the error
     error = max(abs(FUN));
      
     end

% Reset of the error for the next input angle
error = 2*tolerance;

% Saving values for later output
output(i,:) = guess;

end
first_th2 = output(1);  %getting first value of theta2
first_th3 = output(1,2);  %the same for theta 3
final_p = output(100);   %final value to keep still in rotation
%% specifiactions of the crank slider portion
r1 = 4; r2 = 5; r3 = 11.57; r3a = r3-5; %link lengths
P1 = [0,0];    %ground coordinate of crank
P4 = [13,2.43239];  %coordinate of pinion ground
radii_p = 1.43239;  %radius of pinion
tilt_angle = 30*deg2rad; %specified angle 
theta2 = linspace(0,tilt_angle);   %input angle) 
ini_th2 = theta2(1);
alfa_list = []   %initializing alpha list
theta3_list = []   %initializing theta3 list
awning_length = []  %awning list
%% specifications of the fourbar simulation:
% point specifications for the fourbar on the stem
RS1x = RS1*cos(Stheta1);
RS1y = RS1*sin(Stheta1);
RS2x = RS2*cos(output(1:100));
RS2y = RS2*sin(output(1:100));
RS3x = RS3*cos(output(1:100,2)');
RS3y = RS3*sin(output(1:100,2)');
RS4x = RS4*cos(Stheta4);
RS4y = RS4*sin(Stheta4);
RS3xt = RS3EXT*cos(output(1:100,2)');   %used for the awning out of loop extension
RS3yt = RS3EXT*sin(output(1:100,2)');   %used as stated above
%% starting the simulation of the fourbar 
for i = 1:length(output(1:i))
    %% end of shaft points/ grounds for four bar
    r3x = P4(1) + r3*cos((pi/2));  %tip of the link
    r3y = P4(2) + r3*sin((pi/2));
    r3xa = P4(1) + r3a*cos((pi/2));  %shorter link
    r3ya = P4(2) + r3a*sin((pi/2));
    
    %% awning four bar simulation;
    %plotting the ground link to stem
    plot([r3xa r3x],[r3ya r3y],'-ro', 'LineWidth', 2); hold on;
     %plotting the input link r4
    plot([r3xa r3xa+RS4x(i)],[r3ya r3ya+RS4y(i)],'-ro', 'LineWidth', 2); hold on;
     %plotting link r3 
    plot([r3xa+RS4x(i) r3xa+RS4x(i)+RS3x(i)],[r3ya+RS4y(i) r3ya+RS4y(i)+RS3y(i) ],'-ro', 'LineWidth', 2); hold on;
     % Plotting link r2
    plot([r3x r3x + RS2x(i)],[r3y r3y + RS2y(i)],'-ro', 'LineWidth', 2); hold on;
    %plotting the awning extension
    plot([r3xa+RS4x(i)+RS3x(i) r3xa+RS4x(i)+RS3x(i)+RS3xt(i)],...
        [r3ya+RS4y(i)+RS3y(i) r3ya+RS4y(i)+RS3y(i)+RS3yt(i)],'-ro', 'LineWidth', 2); hold on;
    %plotting the awning itelf(will me in green)
    plot([r3x r3xa+RS4x(i)+RS3x(i)+RS3xt(i)],...
        [r3y r3ya+RS4y(i)+RS3y(i)+RS3yt(i)],'-go', 'LineWidth', 2); hold on;
    %gathering awning length data vs input
    awning = (sqrt((r3xa+RS4x(i)+RS3x(i)+RS3xt(i))-r3x)^2 - ((r3ya+RS4y(i)+RS3y(i)+RS3yt(i))-r3y)^2);
    awning_length = [awning_length awning]
    
    %% slider crank stand still poistion:
    plot(P1(1),P1(2),'bO','LineWidth',2)   %location of input ground
    plot(P4(1),P4(2),'bO','LineWidth',2)  %location of pinion ground
    alfa = asin(r1*sin(ini_th2)/r2);    %angle for slider end point
    
    r1x = r1*cos(ini_th2);  %x coordinate of crank end
    r1y = r1*sin(ini_th2);  %y coordinate of crank end
    %coupler slider link
    r2x = r1*cos(ini_th2) + r2*cos(alfa);  %x coordinate of slider end
    r2y = 0;      %y coordinate of slider end
    rack_change = 9 - r2x;   %length change to find out arc length traveled
    theta3 = rack_change/radii_p %formula for theta change
    
    
    %% slider crank simulation;
    %plotting the crank slider
    plot([P1(1) r1x r2x],[P1(2) r1y r2y], '-ro', 'LineWidth', 2);%plot of crank slider
    hold on
    plot([P4(1) r3xa r3x],[P4(2) r3ya r3y],'-ro','LineWidth',3); %plot of awning stem 
    hold off
    
    %plotting the rack
    rectangle('Position', [r2x r2y 9 1], 'Facecolor', 'y',...
        'Linewidth', 2, 'LineStyle',':'); 
    
    %plotting the pinion
    viscircles(P4, radii_p, 'LineStyle',':',...
        'EdgeColor','r');   %plotting the pinion
    
    axis ([-2 30 -2 20])
    xlim([-2,30]);
    ylim([-2,20]);
    getframe
    hold off
    %% new loop for sun blockage

    if i == 100

        for s = theta2
            plot(P1(1),P1(2),'bO','LineWidth',2)   %location of input ground
            plot(P4(1),P4(2),'bO','LineWidth',2)  %location of pinion ground
            alfa = asin(r1*sin(s)/r2);    %angle for slider end point
            alfa_list = [alfa_list alfa];    %updating alfa_list
    
            %input link
            r1x = r1*cos(s);  %x coordinate of crank end
            r1y = r1*sin(s);  %y coordinate of crank end
    
            %coupler slider link
            r2x = r1*cos(s) + r2*cos(alfa);  %x coordinate of slider end
            r2y = 0;      %y coordinate of slider end
    
            rack_change = 9 - r2x;   %length change to find out arc length traveled
            theta3 = rack_change/radii_p %formula for theta change
            theta3_list = [theta3_list theta3];
            
            r3x = P4(1) + r3*cos((pi/2)-theta3); %restating stem points to update
            r3y = P4(2) + r3*sin((pi/2)-theta3);
            r3xa = P4(1) + r3a*cos((pi/2)-theta3);
            r3ya = P4(2) + r3a*sin((pi/2)-theta3);
            %% simulation of the crank slider
            %plotting the crank slider
            plot([P1(1) r1x r2x],[P1(2) r1y r2y], '-ro', 'LineWidth', 2);%plot of crank slider
            hold on
            plot([P4(1) r3xa r3x],[P4(2) r3ya r3y],'-ro','LineWidth',3); %plot of awning stem 
            hold off
    
            %plotting the rack
            rectangle('Position', [r2x r2y 9 1], 'Facecolor', 'y',...
            'Linewidth', 2, 'LineStyle',':'); hold on;
    
            %plotting the pinion
            viscircles(P4, radii_p, 'LineStyle',':',...
            'EdgeColor','r'); hold on;   %plotting the pinion

            %% specifications of the fourbar simulation:
            %end points of the fourbar with angle constraint
            RS1x = RS1*cos(Stheta1-theta3);
            RS1y = RS1*sin(Stheta1-theta3);
            RS2x = RS2*cos(output(1:100)-theta3);
            RS2y = RS2*sin(output(1:100)-theta3);
            RS3x = RS3*cos(output(1:100,2)-theta3');
            RS3y = RS3*sin(output(1:100,2)-theta3');
            RS4x = RS4*cos(Stheta4-theta3);
            RS4y = RS4*sin(Stheta4-theta3);
            RS3xt = RS3EXT*cos(output(1:100,2)-theta3');   %used for the awning out of loop extension
            RS3yt = RS3EXT*sin(output(1:100,2)-theta3');   %used as stated above
            %% awning four bar simulation stand still;
            %keep the awning still for rotation and constrain to rotation
            %plotting the ground link to stem
            plot([r3xa r3x],[r3ya r3y],'-ro', 'LineWidth', 2); hold on;
            %plotting the input link r4
            plot([r3xa r3xa+RS4x(i)],[r3ya r3ya+RS4y(i)],'-ro', 'LineWidth', 2); hold on;
            %plotting link r3 
            plot([r3xa+RS4x(i) r3xa+RS4x(i)+RS3x(i)],[r3ya+RS4y(i) r3ya+RS4y(i)+RS3y(i) ],'-ro', 'LineWidth', 2); hold on;
            %Plotting link r2
            plot([r3x r3x + RS2x(i)],[r3y r3y + RS2y(i)],'-ro', 'LineWidth', 2); hold on;
            %plotting the awning extension
            plot([r3xa+RS4x(i)+RS3x(i) r3xa+RS4x(i)+RS3x(i)+RS3xt(i)],...
            [r3ya+RS4y(i)+RS3y(i) r3ya+RS4y(i)+RS3y(i)+RS3yt(i)],'-ro', 'LineWidth', 2); hold on;
            %plotting the awning itelf(will be in green)
            plot([r3x r3xa+RS4x(i)+RS3x(i)+RS3xt(i)],...
            [r3y r3ya+RS4y(i)+RS3y(i)+RS3yt(i)],'-go', 'LineWidth', 2); hold on;
        
            axis ([-2 30 -2 20])
            xlim([-2,30]);
            ylim([-2,20]);
            getframe
            hold off
        end
    %% final figures
    %plotting crank slider section   
    subplot(2,2,1)
    %figure(2)
    plot(theta2, alfa_list);
    hold on
    plot (theta2, theta3_list)
    title('Crank Input vs Slider and Stem')
    xlabel('\theta_2 [rad]')
    ylabel('\theta_3 ,\alpha [rad]')  
    legend('\alpha','\theta_3')
    
    %plotting the awning angle changes vs input
    subplot(2,2,2)
    %figure(3)
    plot(Stheta4, output(1:100), 'm-'); hold on;
    plot(Stheta4, output(1:100,2),'g-'); hold on;
    title('Awning Input vs \theta_6 vs \theta_5')
    xlabel('\theta_4 [rad]')
    ylabel('\theta_5 ,\theta_6 [rad]')  
    legend('\theta_6',' \theta_5')
    
    %plotting the length of awning 
    subplot(2,2,[3,4])
    %figure(4)
    plot(Stheta4, awning_length, 'm-'); hold on;
    title('Awning length vs angle input')
    xlabel('\theta_4 [rad]')
    ylabel('length[m]')  
    
    end


end