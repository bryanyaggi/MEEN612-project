%% Load the robot from a urdf
robot = importrobot('Manipulator\urdf\Manipulator.urdf');
show(robot, 'visuals', 'on', 'collision', 'off');

%% Animate the robot
robot.DataFormat='column';
robot.Gravity = [0 0 9.81]';
q = homeConfiguration(robot);
qddot = forwardDynamics(robot, q, [], []);

tic()

newVid = VideoWriter("upflop.mp4", 'MPEG-4'); % New
newVid.FrameRate = 30;
newVid.Quality = 100;
open(newVid);

initial_angle = 10 * pi / 180;
q = initial_angle * [1 1 1 1 1 1 1]';
qd = [0 0 0 0 0 0 0]';
tau = [0 0 0 0 0 0 0]';

show(robot, q, 'visuals', 'on', 'collision', 'off');

dt = 0.001; % seconds
loops = 1000;
for i = 1:loops
    show(robot, q, 'visuals', 'on', 'collision', 'off');
    q = q + qd * dt;
    tau = -0.01 * qd;
    qd = qd + dt * robot.forwardDynamics(q, qd, tau);

    drawnow
    writeVideo(newVid,frame2im(getframe(gcf)))
end
close(newVid)
toc()
