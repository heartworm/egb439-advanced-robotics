%% Constants
clear; close all; clc;
const.IP_ROBOT = '172.19.232.102';
const.IP_LOCALISER = '172.19.232.12';
const.DIM_ROBOT = 0.15;
const.DIM_IMAGE = 500;
const.DIM_FIELD = 1.96;

%% 
robot = Robot();
robot.setup(IP_ROBOT, IP_LOCALISER);
