startScript;                                    % Clears screen and variables, except allows CI testing
clear PID
load MagnetoShield_Models_Greybox_SS                            % Include linearized state-space model
load MagnetoShield_PID_Data
disturbance=1;                                                  % Adds disturbance to simulation (1)

Ts=5E-3;                                                        % [s] sampling for discrete control
R=[14.0,13.0,15.0,16.5,13.5,13.0]';                             % [mm] Reference levels for the simulation
T=1000;                                                         % [samples] Section length for each reference level

y0= 14.3;                                                       % [mm] Linearization point based on the experimental identification
i0= 0.0219;                                                     % [A] Linearization point based on the experimental identification
u0= 4.6234;                                                     % [V] Linearization point based on the experimental identification

ra=(R-y0)/1000;                                                 % [mm] Adjusted reference levels for the linearization point
t=0:Ts:(T*length(R)-1)*Ts;                                      % [s] Time vector for the simulation
umin=0;                                                         % [V] Lower input constraint
umax=12;                                                        % [V] Upper input constraint
i=1;                                                            % [-] Section counter for the simulation
r=ra(1);                                                        % [mm] First reference to start simulation

%% Discretization
modeld=c2d(model,Ts);                                           % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0 0];                                                      % C for introducing an integration component

%% Model augmentation by integration
[ny nx]=size(C);                                                % Sizing C
[nx nu]=size(B);                                                % Sizing B
Ai=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Ci=[zeros(ny,nx) C];                                            % Augmenting C by the integrator

%% LQ design
Qlq=diag([1E2 10 10 1]);                                        % State penalty matrix
Rlq=1E-4;                                                       % Input penalty matrix
K=dlqr(Ai,Bi,Qlq,Rlq);                                          % LQ gain

%% Simulation

x0=[0 0 0]';                                                    % Initial condition
X=x0;                                                           % State logging (initialization)
xI=0;                                                           % Integrator initialization
y=[17 0]';                                                      % Output logging (initialization)
Y=y;
x1p=0;     
                                                     
PID = PID;                        % Create PID object from PID class

Kp=2.3;                                                         % [V*mm]
Ti=0.1;                                                         % [s]
Td=0.02;                                                        % [s]
PID.setParameters(Kp, Ti, Td, Ts);                              % Feed the constants to PID object % Previous state

for k=1:length(t)-1
    % Experiment (simulation) profile
    Rr(k)=R(i);                                                 % Original reference, just for logging
    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = ra(i);                                              % and use the actual, corrected reference
    end
    
    % Measurement, primitively reconstructing system states from "measurements"     
    x1=y(1,k)/1000-y0/1000;                                     % Compensate for meters and linearization point from direct measurement
    x2=X(2,k);
    x1p=x1;                                                     % Previous position output (state estimate)
    x3=y(2,k)/1000-i0;                                          % Compensate for Amperes and linearization point from direct measurement
    xhat=[x1 x2 x3]';                                           % Final estimate
    
    % Control
    xI(k+1)=xI(k)+(r-x1);                                       % Integrator state
    U2(k)   = PID.compute((r*1000-(y(1,k)-y0)), 0-u0, 10-u0, 0-u0, 10-u0)+u0;        % Compute PID response;
    U(k)    =-K*[xI(k); xhat]+u0;                               % LQ compensation for the integrator augmented linear part



    % Model
    X(:,k+1)=A*X(:,k)+B*(U(k)-u0);                              % System model
    X(1,k+1)=constrain(X(1,k+1),12E-3-y0*1E-3,17E-3-y0*1E-3);   % Cage limits
    X(3,k+1)=constrain(X(3,k+1),-i0,60E-3-i0);                  % No reverse current possible
    if disturbance
        X(:,k+1)=X(:,k+1)+rand(1)*1E-4;                         % Add random position disturbance (e.g. process noise)
    end
    y(1,k+1)=X(1,k)*1000+y0;                                    % Position measurement on the real system
    y(2,k+1)=(X(3,k)+i0)*1000;                                  % Current measurement on the real system
  
end

%% Plotting

plot(U)
hold on
plot(U2)
axis([0,6000,0, 10])
return

figure(1);                                                      % New figure

subplot(2,1,1)                                                  % Top subplot
plot(t(1:end-1),Rr);                                            % Plot reference
hold on                                                         % Draw on top of this
plot(t(1:end-1),y(1,1:end-1));                                  % Plot the output (position)
grid on                                                         % Grid is enabled
xlabel('Time (s)')                                              % X axis label
ylabel('Position (mm)')                                         % Y axis label
%legend('Reference','Position')

subplot(2,1,2)                                                  % Bottom subplot
plot(t(1:end-1),U);                                             % Plot inputs
grid on                                                         % Allow grid
xlabel('Time (s)')                                              % X axis label
ylabel('Input (V)')                                             % Y axis label