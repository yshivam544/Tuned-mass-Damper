function tmd
    % k1=13.98;
    % k1=22.222222;
    k1=752;
    m1=2;
    l=0.03; %perfectly tuned at this length of the pendulum.
    m2=0.8;
    k2=(m2*9.8)/l;

    %building reference
    % kref=13.98;
    % kref=22.222222;
    kref=752;
    mref=2;

    %oscillation amplitude of building remains constant at 0.08m

    TSPAN=linspace(0,30,3000);
    initial_displacement_x1=0;  
    initial_velocity_x1=0;
    initial_displacement_x2=0;
    initial_velocity_x2=0;

    initial_displacement_xref=0;
    initial_velocity_xref=0;

    d=0.5;  %damping factor
    
    z0=[initial_displacement_x1 initial_velocity_x1 initial_displacement_x2 initial_velocity_x2 initial_displacement_xref initial_velocity_xref];
    OPTIONS = odeset('RelTol',1e-9,'AbsTol',1e-12);
    [t,z] = ode45(@odefun,TSPAN,z0,OPTIONS,d,k1,k2,kref,m1,m2,mref);
    % figure(1)
    % plot(TSPAN,z(:,1),'b',TSPAN,z(:,3),'r') 
    % ylabel('x1 in m and x2 in m');xlabel('Time');title('blue line is x1 and red line is x2 ');
    figure(1)
    plot(TSPAN,z(:,1),'b',TSPAN,z(:,3),'r',TSPAN,z(:,5),'g') 
    ylabel('x1, x2 and xref in m');xlabel('Time');title('blue line is x1, red line is x2, green line is xref');
    % figure(2)
    % plot(TSPAN,z(:,2),'b',TSPAN,z(:,4),'r') 
    % ylabel('v1 in m/s and v2 in m/s');xlabel('Time');title('blue line is v1 and red line is v2 ');
    
function zdot= odefun(t,z,d,k1,k2,kref,m1,m2,mref)
    %forced + damped 
    %omega_natural=18
    zdot=[z(2);(-(k1/m1)*(z(1)))+((k2/m1)*(z(3)-z(1)))+(2*cos(18*t))-((d/m1)*z(2)); 
    z(4);(-(k2/m2)*(z(3)-z(1)))
    z(6);(-(kref/mref)*(z(5)))+(2*cos(18*t))-((d/m1)*z(6))];

    %natural + damped 
    % zdot=[z(2);(-(k1/m1)*(z(1)))+((k2/m1)*(z(3)-z(1)))-((d/m1)*z(2)); 
    % z(4);(-(k2/m2)*(z(3)-z(1)))
    % z(6);(-(kref/mref)*(z(5)))-((d/mref)*z(6))];

    %forced + undamped
    % zdot=[z(2);(-(k1/m1)*(z(1)))+((k2/m1)*(z(3)-z(1)))+(2*cos(18*t)); 
    % z(4);(-(k2/m2)*(z(3)-z(1)))
    % z(6);(-(kref/mref)*(z(5)))+(2*cos(18*t))];

    %natural + undamped
    % zdot=[z(2);(-(k1/m1)*(z(1)))+((k2/m1)*(z(3)-z(1))); 
    % z(4);(-(k2/m2)*(z(3)-z(1)))
    % z(6);(-(kref/mref)*(z(5)))];