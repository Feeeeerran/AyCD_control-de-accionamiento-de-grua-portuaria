function x  = Read_OPC_Func(y)
% variables
persistent init_Server;
persistent init_Nodes;
persistent uaClient;
persistent Var_Node_In;
persistent Var_Node_Out;
persistent testVal;
% initialize variables
disp("Entro");
if (isempty(init_Server))
    disp("Entro2");
    testVal = [];
    init_Server = 0;
    init_Nodes = 0;
end
% OPC UA server (PLC) address and connecting client (Simulink) to the server
if init_Server == 0
    disp("Entro3");
    init_Server = 1;
    uaClient = opcua('localhost',4840);
    connect(uaClient);
end
% define variable nodes in the server
if uaClient.isConnected == 1 && init_Nodes == 0
    disp("Entro4");
    init_Nodes = 1;
    disp("Estamos aca 2");
    % read out the variables of the OPC UA server
    Var_Node_In = [opcuanode(4,'|var|CODESYS Control Win V3 x64.Application.GVL.variable1',uaClient);
        opcuanode(4,'|var|CODESYS Control Win V3 x64.Application.GVL.variable',uaClient)];
    Var_Node_Out= opcuanode(3,"""GVL_OPCUA"".""PLC_WriteVar""",uaClient);
end
% read and write variables of the server
if uaClient.isConnected == 1 && init_Nodes == 1
    disp("Entro5");
    % read "fanForce" value from server and store in "val"
    [val, ~, ~] = readValue(uaClient, Var_Node_In);
    % assign input y of the function to "currentAngle" variable
    %writeValue(uaClient, Var_Node_Out, y);
    % assign "val" to variable "testVal"
    testVal = [val{:}];
end
% assign "fanForce" ("testVal") value to the output x of the function
disp("Entro6");
x = double(testVal);
end