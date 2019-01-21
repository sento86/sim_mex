
clear all;
clc;

%% Directory setup

dir='/home/idf/ros/magv_simulator/magv/sim_mex/src/';
cd(fullfile(dir));

%%

mex -v...
    -I/home/idf/ros/magv_simulator/magv/sim_mex/src/...
    -I/home/idf/ros/magv_simulator/magv/sim_mex/src/shared/...
    -L/usr/lib/x86_64-linux-gnu -L/home/idf/libraries/PhysX-3.3.1/Lib/linux64...
    /home/idf/ros/magv_simulator/magv/ide/codelite/Release/sim_mex/libsim_mex.so...
    -lm -lc -lpthread -lboost_system -lOIS -ltinyxml...
    main_mex.cpp -output sim_bus

%%
