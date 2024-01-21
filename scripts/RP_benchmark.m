%% Formation benchmark
clc
close all
clear variables

%% Problem 1
N=15;
R=1;
tic
[D,E,P]= problem1(N,R,0);
t = toc;

% J1
Ed = 0;
for i=1:length(D)
    Ed = Ed + 1/D(i)^2;
end
J1 = 2.4283*t + 1.0*(max(D)-min(D)) + 0.0079 * Ed
%% Files Generation
if N<10
    folder_name = strcat('ControlFormation0',int2str(N),'_files');
else
    folder_name = strcat('ControlFormation',int2str(N),'_files');
end
folder = dir(fullfile(pwd,folder_name));
if(~isempty(folder))
   cd(folder_name) 
else
    status = mkdir(folder_name);
    cd(folder_name)
end

robot_name = string(N);
for i=1:N
    if i<10
        robot_name(i,1) = strcat('dron0',int2str(i));
    else
        robot_name(i,1) = strcat('dron',int2str(i));
    end
end

relationship = string(N);
for i = 1:N
    aux = string();
    for z = 1:length(E)
        if E(z,1) == i
            aux = (aux+robot_name(E(z,2))+'_'+num2str(D(z,1))+', ');
        end
        if E(z,2) == i
            aux = (aux+robot_name(E(z,1))+'_'+num2str(D(z,1))+', ');
        end
    end
    relationship(i,1) = robot_name(i);
    relationship(i,2) = extractBetween(aux,1,strlength(aux)-2);
end

% Default Configuration file
fid = fopen('default_config.yaml', 'wt' );
fprintf(fid, '# Default Robotic Park Formation Control Benchmark\n\n');
fprintf(fid, 'Operation:\n');
fprintf(fid, '  mode: virtual     # physical, virtual, hybrid\n');
fprintf(fid, '  tool: Webots              # Gazebo, Webots\n');
fprintf(fid, '  world: default_world.wbt \n');
fprintf(fid, '\nExperience:\n');
fprintf(fid, '  type: formation           # identification, formation, navigation\n');
fprintf(fid, '\nArchitecture: \n');
fprintf(fid, '  mode: distributed   # centralized, distributed_ros2, distributed\n');
fprintf(fid, '  node: \n');
fprintf(fid, '    executable: centralized_formation_controller\n');
fprintf(fid, '    name: formation_controller\n');
fprintf(fid, '    pkg: uned_swarm_task\n');
fprintf(fid, '\nCPU_Monitoring: \n');
fprintf(fid, '  enable: True\n');
fprintf(fid, '  node: \n');
fprintf(fid, '    executable: measure_process\n');
fprintf(fid, '    name: benchmark\n');
fprintf(fid, '    pkg: measure_process_ros2_pkg\n');
fprintf(fid, '  processes: webots-bin, driver, ros2, swarm_driver, rviz2, centralized_for\n');
fprintf(fid, '\nInterface: \n');
fprintf(fid, '  enable: True\n');
fprintf(fid, '  rviz2: \n');
fprintf(fid, '    enable: True\n');
fprintf(fid, '    node: \n');
fprintf(fid, '      executable: rviz2\n');
fprintf(fid, '      name: rviz2\n');
fprintf(fid, '      pkg: rviz2\n');
fprintf(fid, '    file: default_config.rviz\n');
fprintf(fid, '  rqt: \n');
fprintf(fid, '    enable: True\n');
fprintf(fid, '    node: \n');
fprintf(fid, '      executable: rqt_gui\n');
fprintf(fid, '      name: interface\n');
fprintf(fid, '      pkg: rqt_gui\n');
fprintf(fid, '    file: \n');
fprintf(fid, '  own: \n');
fprintf(fid, '    enable: False\n');
fprintf(fid, '    node:\n');
fprintf(fid, '      name:\n');
fprintf(fid, '      pkg:\n');
fprintf(fid, '    file:\n');
fprintf(fid, '\nData_Logging: \n');
fprintf(fid, '  enable: False\n');
fprintf(fid, '  all: True\n');
fprintf(fid, '  name: date\n');
topics = '/ros2_cpu /cpu_stats';
for i = 1:N
    topics = strcat(topics,{' /'},robot_name(i),{'/local_pose'});
    topics = strcat(topics,{' /'},robot_name(i),{'/global_error'});
    for z = 1:length(E)
        if E(z,1) == i
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,2)),{'/data'});
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,2)),{'/error'});
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,2)),{'/iae'});
        end
        if E(z,2) == i
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,1)),{'/data'});
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,1)),{'/error'});
            topics = strcat(topics,{' /'},robot_name(i),{'/'},robot_name(E(z,1)),{'/iae'});
        end
    end
    topics = strcat(topics,{' /'},robot_name(i),{'/origin/data'});
    topics = strcat(topics,{' /'},robot_name(i),{'/origin/error'});
    topics = strcat(topics,{' /'},robot_name(i),{'/origin/iae'});
end
fprintf(fid, '  topics: %s\n',topics);
fprintf(fid, '\nRobots:\n');
for i = 1:N
    if i<10
        id = strcat('Robot0',int2str(i));
    else
        id = strcat('Robot',int2str(i));
    end
    fprintf(fid, '  %s:\n', id);
    fprintf(fid, '    type: virtual\n');
    fprintf(fid, '    name: %s\n',robot_name(i));
    fprintf(fid, '    control_mode: HighLevel\n');
    fprintf(fid, '    positioning: Intern\n');
    fprintf(fid, '    pose: %s, %s, %s\n', num2str(P(i,1)*1.2+(rand-0.5)*0.1,'%.2f'), num2str(P(i,2)*1.2+(rand-0.5)*0.1,'%.2f'), num2str(0.7,'%.2f'));
    if i<10
        fprintf(fid, '    uri: radio://0/80/2M/E7E7E7E70%d\n',i);
    else
        fprintf(fid, '    uri: radio://0/80/2M/E7E7E7E7%d\n',i);
    end
    fprintf(fid, '    controller:\n');
    fprintf(fid, '      type: pid\n');
    fprintf(fid, '      enable: True\n');
    fprintf(fid, '      protocol: Continuous\n');
    fprintf(fid, '      period: 0.01\n');
    fprintf(fid, '      threshold:\n');
    fprintf(fid, '        type: Constant\n');
    fprintf(fid, '        co: 0.01\n');
    fprintf(fid, '        ai: 0.0\n');
    fprintf(fid, '    communication:\n');
    fprintf(fid, '      type: EventBased\n');
    fprintf(fid, '      threshold:\n');
    fprintf(fid, '        type: Constant\n');
    fprintf(fid, '        co: 0.01\n');
    fprintf(fid, '        ai: 0.0\n');
    fprintf(fid, '    local_pose:\n');
    fprintf(fid, '      enable: True\n');
    fprintf(fid, '      path: False\n');
    fprintf(fid, '      T: 100\n');
    fprintf(fid, '    local_twist:\n');
    fprintf(fid, '      enable: False\n      T: 20\n');
    fprintf(fid, '    data_attitude:\n');
    fprintf(fid, '      enable: False\n      T: 20\n');
    fprintf(fid, '    data_rate: \n');
    fprintf(fid, '      enable: False\n      T: 20\n');
    fprintf(fid, '    data_motor: \n');
    fprintf(fid, '      enable: False\n      T: 20\n');
    fprintf(fid, '    data:\n');
    fprintf(fid, '      enable: False\n      T: 20\n');
    fprintf(fid, '    mars_data:\n');
    fprintf(fid, '      enable: True\n      T: 50\n');
    fprintf(fid, '    task: \n');
    fprintf(fid, '      enable: True\n');
    fprintf(fid, '      T: 100\n');
    fprintf(fid, '      Onboard: True\n');
    fprintf(fid, '      controller:\n');
    fprintf(fid, '        type: gradient\n');
    fprintf(fid, '        protocol: Continuous\n');
    fprintf(fid, '        period: 0.1\n');
    fprintf(fid, '        upperLimit: 0.1\n');
    fprintf(fid, '        lowerLimit: -0.1\n');
    fprintf(fid, '        gain: 0.25\n');
    fprintf(fid, '        threshold:\n');
    fprintf(fid, '            type: Constant\n');
    fprintf(fid, '            co: 0.01\n');
    fprintf(fid, '            ai: 0.0\n');
    fprintf(fid, '      role: consensus\n');
    fprintf(fid, '      type: distance\n');
    fprintf(fid, '      relationship: %s, origin_%0.1f\n\n',relationship(i,2),R);
end
fprintf(fid, 'Supervisor:\n');
fprintf(fid, '  enable: True\n');
fprintf(fid, '  node: \n');
fprintf(fid, '    executable: supervisor_node\n');
fprintf(fid, '    name: supervisor\n');
fprintf(fid, '    pkg: mars_supervisor_pkg\n');
fprintf(fid, '    file: default_topics.yaml\n\n');
fprintf(fid, 'Other:\n');
fprintf(fid, '  Agent00:\n');
fprintf(fid, '    executable: test\n');
fprintf(fid, '    name: test\n');
fprintf(fid, '    pkg: test\n');
fprintf(fid, '    file: test_file\n');

fclose(fid);

% Webots World
fid = fopen('default_world.wbt', 'wt' );
fprintf(fid, '#VRML_SIM R2023b utf8\n\n');
fprintf(fid, 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"\n');
fprintf(fid, 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/Floor.proto"\n');
fprintf(fid, 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n');
fprintf(fid, 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto"\n');
fprintf(fid, 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/appearances/protos/Parquetry.proto"\n\n');
fprintf(fid, 'WorldInfo {\n\tinfo [\n');
fprintf(fid, '\t\t "Robotic Park World. Formation Control Benchmark. Case N:%s and R:%s"\n\t]\n',int2str(N), num2str(R,'%.1f'));
fprintf(fid, '\tbasicTimeStep 20\n}\n');
fprintf(fid, 'Viewpoint {\n');
fprintf(fid, '\tfieldOfView 1\n');
fprintf(fid, '\torientation 0.08257814004826021 0.7347446275807098 -0.6732987323821825 0.7219016991902246\n');
fprintf(fid, '\tposition -6.686523789804856 3.246953734178069 5.0217815008034\n');
fprintf(fid, '}\n');
fprintf(fid, 'TexturedBackground {}\n');
fprintf(fid, 'TexturedBackgroundLight {}\n');
fprintf(fid, 'DirectionalLight {\n');
fprintf(fid, '    ambientIntensity 1\n');
fprintf(fid, '    direction 6 -16 -10\n');
fprintf(fid, '    intensity 4\n}\n');
fprintf(fid, 'Floor {\n');
fprintf(fid, '    translation -1 0 0\n');
fprintf(fid, '    size 8 6\n');
fprintf(fid, '    appearance Parquetry {\n');
fprintf(fid, '        type "chequered"\n');
fprintf(fid, '        IBLStrength 0\n');
fprintf(fid, '    }\n}\n');
fprintf(fid, 'Solid {\n');
fprintf(fid, '    rotation 0 0 1 3.14159\n');
fprintf(fid, '    children [\n');
fprintf(fid, '        CadShape {\n');
fprintf(fid, '        url [\n');
fprintf(fid, '            "meshes/RoboticPark.dae"\n');
fprintf(fid, '        ]\n        }\n    ]\n');
fprintf(fid, '    contactMaterial ""\n');
fprintf(fid, '    boundingObject Mesh {\n');
fprintf(fid, '         url [\n');
fprintf(fid, '        "meshes/RoboticPark_new.dae"\n');
fprintf(fid, '        ]\n    }\n');
fprintf(fid, '    radarCrossSection 1\n}\n\n');
for i=1:N
    fprintf(fid, 'Crazyflie {\n');
    fprintf(fid, '\ttranslation %s %s 0.015\n', num2str(P(i,1)*1.2+(rand-0.5)*0.1,'%.2f'), num2str(P(i,2)*1.2+(rand-0.5)*0.1,'%.2f'));
    fprintf(fid, '\tname "%s"\n',relationship(i,1));
    fprintf(fid, '\tcontroller "<extern>"\n');
    if contains(relationship(i,1),'dron01')
        fprintf(fid, '  supervisor TRUE\n');
    end
    fprintf(fid, '  extensionSlot [\n');
    fprintf(fid, '    InertialUnit {\n    }\n  ]\n}\n\n');
end
fclose(fid);

% RVIZ2 Configuration File

fid = fopen('default_config.rviz', 'wt' );

fprintf(fid, 'Panels:\n');
fprintf(fid, '  - Class: rviz_common/Displays\n');
fprintf(fid, '    Help Height: 78\n');
fprintf(fid, '    Name: Displays\n');
fprintf(fid, '    Property Tree Widget:\n');
fprintf(fid, '      Expanded:\n');
fprintf(fid, '      Splitter Ratio: 0.5\n');
fprintf(fid, '    Tree Height: 549\n');
fprintf(fid, '  - Class: rviz_common/Selection\n');
fprintf(fid, '    Name: Selection\n');
fprintf(fid, '  - Class: rviz_common/Tool Properties\n');
fprintf(fid, '    Expanded:\n');
fprintf(fid, '      - /2D Goal Pose1\n');
fprintf(fid, '      - /Publish Point1\n');
fprintf(fid, '    Name: Tool Properties\n');
fprintf(fid, '    Splitter Ratio: 0.5886790156364441\n');
fprintf(fid, '  - Class: rviz_common/Views\n');
fprintf(fid, '    Expanded:\n');
fprintf(fid, '      - /Current View1\n');
fprintf(fid, '    Name: Views\n');
fprintf(fid, '    Splitter Ratio: 0.5\n');
fprintf(fid, '  - Class: rviz_common/Time\n');
fprintf(fid, '    Experimental: false\n');
fprintf(fid, '    Name: Time\n');
fprintf(fid, '    SyncMode: 0\n');
fprintf(fid, '    SyncSource: ""\n');
fprintf(fid, 'Visualization Manager:\n');
fprintf(fid, '  Class: ""\n');
fprintf(fid, '  Displays:\n');
fprintf(fid, '    - Alpha: 0.5\n');
fprintf(fid, '      Cell Size: 1\n');
fprintf(fid, '      Class: rviz_default_plugins/Grid\n');
fprintf(fid, '      Color: 160; 160; 164\n');
fprintf(fid, '      Enabled: true\n');
fprintf(fid, '      Line Style:\n');
fprintf(fid, '        Line Width: 0.029999999329447746\n');
fprintf(fid, '        Value: Lines\n');
fprintf(fid, '      Name: Grid\n');
fprintf(fid, '      Normal Cell Count: 0\n');
fprintf(fid, '      Offset:\n');
fprintf(fid, '        X: 0\n        Y: 0\n        Z: 0\n');
fprintf(fid, '      Plane: XY\n');
fprintf(fid, '      Plane Cell Count: 10\n');
fprintf(fid, '      Reference Frame: <Fixed Frame>\n');
fprintf(fid, '      Value: true\n');
fprintf(fid, '    - Alpha: 1\n');
fprintf(fid, '      Class: rviz_default_plugins/RobotModel\n');
fprintf(fid, '      Collision Enabled: false\n');
fprintf(fid, '      Description File: src/uned_crazyflie_ros_pkg/uned_crazyflie_config/model/urdf/RoboticPark_simple.urdf\n');
fprintf(fid, '      Description Source: File\n');
fprintf(fid, '      Description Topic:\n');
fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
fprintf(fid, '        Value: ""\n');
fprintf(fid, '      Enabled: true\n');
fprintf(fid, '      Links:\n');
fprintf(fid, '        All Links Enabled: true\n');
fprintf(fid, '        Expand Joint Details: false\n');
fprintf(fid, '        Expand Link Details: false\n');
fprintf(fid, '        Expand Tree: false\n');
fprintf(fid, '        Link Tree Style: Links in Alphabetic Order\n');
fprintf(fid, '        base_link:\n');
fprintf(fid, '          Alpha: 1\n');
fprintf(fid, '          Show Axes: false\n');
fprintf(fid, '          Show Trail: false\n');
fprintf(fid, '          Value: true\n');
fprintf(fid, '      Mass Properties:\n');
fprintf(fid, '        Inertia: false\n');
fprintf(fid, '        Mass: false\n');
fprintf(fid, '      Name: RobotModel\n');
fprintf(fid, '      TF Prefix: RoboticPark\n');
fprintf(fid, '      Update Interval: 0\n');
fprintf(fid, '      Value: true\n');
fprintf(fid, '      Visual Enabled: true\n');
for i=1:N
    fprintf(fid, '    - Alpha: 1\n');
    fprintf(fid, '      Class: rviz_default_plugins/RobotModel\n');
    fprintf(fid, '      Collision Enabled: false\n');
    fprintf(fid, '      Description File: src/uned_crazyflie_ros_pkg/uned_crazyflie_config/model/urdf/crazyflie_simple.urdf\n');
    fprintf(fid, '      Description Source: File\n');
    fprintf(fid, '      Description Topic:\n');
    fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
    fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
    fprintf(fid, '        Value: ""\n');
    fprintf(fid, '      Enabled: true\n');
    fprintf(fid, '      Links:\n');
    fprintf(fid, '        All Links Enabled: true\n');
    fprintf(fid, '        Expand Joint Details: false\n');
    fprintf(fid, '        Expand Link Details: false\n');
    fprintf(fid, '        Expand Tree: false\n');
    fprintf(fid, '        Link Tree Style: Links in Alphabetic Order\n');
    fprintf(fid, '        base_link:\n');
    fprintf(fid, '          Alpha: 1\n');
    fprintf(fid, '          Show Axes: false\n');
    fprintf(fid, '          Show Trail: false\n');
    fprintf(fid, '          Value: true\n');
    fprintf(fid, '      Mass Properties:\n');
    fprintf(fid, '        Inertia: false\n');
    fprintf(fid, '        Mass: false\n');
    fprintf(fid, '      Name: %s\n',relationship(i,1));
    fprintf(fid, '      TF Prefix: %s\n',relationship(i,1));
    fprintf(fid, '      Update Interval: 0\n');
    fprintf(fid, '      Value: true\n');
    fprintf(fid, '      Visual Enabled: true\n');
    fprintf(fid, '    - Alpha: 1\n');
    fprintf(fid, '      Axes Length: 1\n');
    fprintf(fid, '      Axes Radius: 0.10000000149011612\n');
    fprintf(fid, '      Class: rviz_default_plugins/Pose\n');
    fprintf(fid, '      Color: 255; 25; 0\n');
    fprintf(fid, '      Enabled: true\n');
    fprintf(fid, '      Head Length: 0.10000000149011612\n');
    fprintf(fid, '      Head Radius: 0.05000000074505806\n');
    fprintf(fid, '      Name: %s-local_pose\n',relationship(i,1));
    fprintf(fid, '      Shaft Length: 0.25\n');
    fprintf(fid, '      Shaft Radius: 0.019999999552965164\n');
    fprintf(fid, '      Shape: Arrow\n');
    fprintf(fid, '      Topic:\n');
    fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
    fprintf(fid, '        Filter size: 10\n');
    fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
    fprintf(fid, '        Value: /%s/local_pose\n',relationship(i,1));
    fprintf(fid, '      Value: true\n');
    fprintf(fid, '    - Alpha: 1\n');
    fprintf(fid, '      Axes Length: 1\n');
    fprintf(fid, '      Axes Radius: 0.10000000149011612\n');
    fprintf(fid, '      Class: rviz_default_plugins/Pose\n');
    fprintf(fid, '      Color: 255; 163; 72\n');
    fprintf(fid, '      Enabled: true\n');
    fprintf(fid, '      Head Length: 0.15000000596046448\n');
    fprintf(fid, '      Head Radius: 0.05000000074505806\n');
    fprintf(fid, '      Name: %s-goal_pose\n',relationship(i,1));
    fprintf(fid, '      Shaft Length: 0.25\n');
    fprintf(fid, '      Shaft Radius: 0.02500000037252903\n');
    fprintf(fid, '      Shape: Arrow\n');
    fprintf(fid, '      Topic:\n');
    fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
    fprintf(fid, '        Filter size: 10\n');
    fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
    fprintf(fid, '        Value: /%s/goal_pose\n',relationship(i,1));
    fprintf(fid, '      Value: true\n');
    fprintf(fid, '    - Alpha: 1\n');
    fprintf(fid, '      Buffer Length: 1\n');
    fprintf(fid, '      Class: rviz_default_plugins/Path\n');
    fprintf(fid, '      Color: 26; 95; 180\n');
    fprintf(fid, '      Enabled: true\n');
    fprintf(fid, '      Head Diameter: 0.30000001192092896\n');
    fprintf(fid, '      Head Length: 0.20000000298023224\n');
    fprintf(fid, '      Length: 0.30000001192092896\n');
    fprintf(fid, '      Line Style: Billboards\n');
    fprintf(fid, '      Line Width: 0.029999999329447746\n');
    fprintf(fid, '      Name: %s-path\n',relationship(i,1));
    fprintf(fid, '      Offset:\n');
    fprintf(fid, '        X: 0\n        Y: 0\n        Z: 0\n');
    fprintf(fid, '      Pose Color: 255; 85; 255\n');
    fprintf(fid, '      Pose Style: None\n');
    fprintf(fid, '      Radius: 0.029999999329447746\n');
    fprintf(fid, '      Shaft Diameter: 0.10000000149011612\n');
    fprintf(fid, '      Shaft Length: 0.10000000149011612\n');
    fprintf(fid, '      Topic:\n');
    fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
    fprintf(fid, '        Filter size: 10\n');
    fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
    fprintf(fid, '        Value: /%s/path\n',relationship(i,1));
    fprintf(fid, '      Value: true\n');
    fprintf(fid, '\n');
end
for i=1:length(E)
    fprintf(fid, '    - Class: rviz_default_plugins/Marker\n');
    fprintf(fid, '      Enabled: true\n');
    aux = robot_name(E(i,1))+'-'+robot_name(E(i,2));
    fprintf(fid, '      Name: %s\n',aux);
    fprintf(fid, '      Namespaces:\n');
    fprintf(fid, '        { }\n');
    fprintf(fid, '      Topic:\n');
    fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
    fprintf(fid, '        Filter size: 10\n');
    fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
    aux = '/'+robot_name(E(i,1))+'/'+robot_name(E(i,2))+'/marker';
    fprintf(fid, '        Value: %s\n',aux);
    fprintf(fid, '      Value: true\n');
end
fprintf(fid, '  Enabled: true\n');
fprintf(fid, '  Global Options:\n');
fprintf(fid, '    Background Color: 80; 111; 93\n');
fprintf(fid, '    Fixed Frame: map\n');
fprintf(fid, '    Frame Rate: 30\n');
fprintf(fid, '  Name: root\n');
fprintf(fid, '  Tools:\n');
fprintf(fid, '    - Class: rviz_default_plugins/Interact\n');
fprintf(fid, '      Hide Inactive Objects: true\n');
fprintf(fid, '    - Class: rviz_default_plugins/MoveCamera\n');
fprintf(fid, '    - Class: rviz_default_plugins/Select\n');
fprintf(fid, '    - Class: rviz_default_plugins/FocusCamera\n');
fprintf(fid, '    - Class: rviz_default_plugins/Measure\n');
fprintf(fid, '      Line color: 128; 128; 0\n');
fprintf(fid, '    - Class: rviz_default_plugins/SetInitialPose\n');
fprintf(fid, '      Covariance x: 0.25\n');
fprintf(fid, '      Covariance y: 0.25\n');
fprintf(fid, '      Covariance yaw: 0.06853891909122467\n');
fprintf(fid, '      Topic:\n');
fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
fprintf(fid, '        Value: /initialpose\n');
fprintf(fid, '    - Class: rviz_default_plugins/SetGoal\n');
fprintf(fid, '      Topic:\n');
fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
fprintf(fid, '        Value: origin/local_pose\n');
fprintf(fid, '    - Class: rviz_default_plugins/PublishPoint\n');
fprintf(fid, '      Single click: true\n');
fprintf(fid, '      Topic:\n');
fprintf(fid, '        Depth: 5\n        Durability Policy: Volatile\n');
fprintf(fid, '        History Policy: Keep Last\n        Reliability Policy: Reliable\n');
fprintf(fid, '        Value: /clicked_point\n');
fprintf(fid, '  Transformation:\n');
fprintf(fid, '    Current:\n');
fprintf(fid, '      Class: rviz_default_plugins/TF\n');
fprintf(fid, '  Value: true\n');
fprintf(fid, '  Views:\n');
fprintf(fid, '    Current:\n');
fprintf(fid, '      Class: rviz_default_plugins/Orbit\n');
fprintf(fid, '      Distance: 4.903817176818848\n');
fprintf(fid, '      Enable Stereo Rendering:\n');
fprintf(fid, '        Stereo Eye Separation: 0.05999999865889549\n');
fprintf(fid, '        Stereo Focal Distance: 1\n');
fprintf(fid, '        Swap Stereo Eyes: false\n');
fprintf(fid, '        Value: false\n');
fprintf(fid, '      Focal Point:\n');
fprintf(fid, '        X: -0.083\n        Y: -0.179\n        Z: 0.560\n');
fprintf(fid, '      Focal Shape Fixed Size: true\n');
fprintf(fid, '      Focal Shape Size: 0.05000000074505806\n');
fprintf(fid, '      Invert Z Axis: false\n');
fprintf(fid, '      Name: Current View\n');
fprintf(fid, '      Near Clip Distance: 0.009999999776482582\n');
fprintf(fid, '      Pitch: 0.7353980541229248\n');
fprintf(fid, '      Target Frame: <Fixed Frame>\n');
fprintf(fid, '      Value: Orbit (rviz)\n');
fprintf(fid, '      Yaw: 2.6654012203216553\n');
fprintf(fid, '    Saved: ~\n');
fprintf(fid, 'Window Geometry:\n');
fprintf(fid, '  Displays:\n');
fprintf(fid, '    collapsed: true\n');
fprintf(fid, '  Height: 846\n');
fprintf(fid, '  Hide Left Dock: true\n');
fprintf(fid, '  Hide Right Dock: false\n');
fprintf(fid, '  QMainWindow State: 000000ff00000000fd0000000400000000000001ae000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073000000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002fb00fffffffb0000000800540069006d00650100000000000004500000000000000000000004b0000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000\n');
fprintf(fid, '  Selection:\n');
fprintf(fid, '    collapsed: false\n');
fprintf(fid, '  Time:\n');
fprintf(fid, '    collapsed: false\n');
fprintf(fid, '  Tool Properties:\n');
fprintf(fid, '    collapsed: false\n');
fprintf(fid, '  Views:\n');
fprintf(fid, '    collapsed: false\n');
fprintf(fid, '  Width: 1200\n');
fprintf(fid, '  X: 64\n  Y: 176\n');
fclose(fid);
cd ..
