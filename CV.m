clear; clc; close all;
image = load("img.mat");
image = image.img;

% dispaly image
% idisp(image);
 
[grass, goal] = DoCV(image);

idisp(grass);