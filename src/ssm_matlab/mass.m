clear; clc;
body = 562.768;
booster = 120;
holder = 13.5;
finholder = 0.7;
fin = 1.5;
wing = 9.29998;

fullbody = body + finholder*4 + fin*4 + wing*4 + booster + holder*4
missile_booster = body + finholder*4 + fin*4 + wing*4 + booster
missile_main = body + finholder*4 + fin*4 + wing*4

force_to_hold_core = missile_main*9.8
force_to_hold_missile = missile_booster*9.8
