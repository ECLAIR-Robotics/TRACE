exec(open("Modified_data/realtime.py").read())
exec(open("Modified_data/ball.dr").read())
#trick.stop(10.4)
#==================================
# Start the variable server client.
#==================================
varServerPort = trick.var_server_get_port();
RobotDisplay_path = os.environ['HOME'] + "/trick_sims/TRACE/SIM_TRACE/PlatformControl.py"
if (os.path.isfile(RobotDisplay_path)) :
    RobotDisplay_cmd = RobotDisplay_path + " " + str(varServerPort) + " &" ;
    print(RobotDisplay_cmd)
    os.system(RobotDisplay_cmd);
else :
    print('Oops! Can\'t find ' + RobotDisplay_path )
