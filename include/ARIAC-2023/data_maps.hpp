#pragma once

#include <string>
#include <map>

namespace j_s_maps
{
    class FloorRobotJS{

        public:

            std::map<std::string, double> rail_positions_ = {
                {"agv1", -4.5},
                {"agv2", -1.2},
                {"agv3", 1.2},
                {"agv4", 4.5},
                {"left_bins", 3},
                {"right_bins", -3}};

            // Joint value targets for kitting stations
            std::map<std::string, double> floor_kts1_js_ = {
                {"linear_actuator_joint", 4.0},
                {"floor_shoulder_pan_joint", 1.57},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> floor_kts2_js_ = {
                {"linear_actuator_joint", -4.0},
                {"floor_shoulder_pan_joint", -1.57},
                {"floor_shoulder_lift_joint", -1.57},
                {"floor_elbow_joint", 1.57},
                {"floor_wrist_1_joint", -1.57},
                {"floor_wrist_2_joint", -1.57},
                {"floor_wrist_3_joint", 0.0}};


            std::map<std::string, double> conv_pick_neg1_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.841327},
                {"floor_elbow_joint", 2.4},
                {"floor_wrist_1_joint", -3.10},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg1_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.9},
                {"floor_elbow_joint", 2.4},
                {"floor_wrist_1_joint", -3.10},
                {"floor_wrist_2_joint", -1.40},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg1_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -1.01},
                {"floor_elbow_joint", 2.4},
                {"floor_wrist_1_joint", -2.97},
                {"floor_wrist_2_joint", -1.45},
                {"floor_wrist_3_joint", 0.0}};
        
            std::map<std::string, double> conv_pick_neg75_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.830000},
                {"floor_elbow_joint", 2.310000},
                {"floor_wrist_1_joint", -3.050000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg75_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.905664},
                {"floor_elbow_joint", 2.324778},
                {"floor_wrist_1_joint", -2.950000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};


            std::map<std::string, double> conv_pick_neg75_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -1.007000},
                {"floor_elbow_joint", 2.337000},
                {"floor_wrist_1_joint", -2.890000},
                {"floor_wrist_2_joint", -1.55},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg50_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.830000},
                {"floor_elbow_joint", 2.247168},
                {"floor_wrist_1_joint", -2.959000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg50_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.885664},
                {"floor_elbow_joint", 2.234778},
                {"floor_wrist_1_joint", -2.890000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg50_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.98700},
                {"floor_elbow_joint", 2.270000},
                {"floor_wrist_1_joint", -2.840000},
                {"floor_wrist_2_joint", -1.55},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg25_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.80000},
                {"floor_elbow_joint", 2.184336},
                {"floor_wrist_1_joint", -3.0000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg25_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.865664},
                {"floor_elbow_joint", 2.184778},
                {"floor_wrist_1_joint", -2.890000},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_neg25_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.957800},
                {"floor_elbow_joint", 2.208000},
                {"floor_wrist_1_joint", -2.850000},
                {"floor_wrist_2_joint", -1.5400},
                {"floor_wrist_3_joint", 0.0}};


            std::map<std::string, double> conv_pick_0_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.80},
                {"floor_elbow_joint", 2.1230},
                {"floor_wrist_1_joint", -2.8686},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_0_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.849646},
                {"floor_elbow_joint", 2.123029},
                {"floor_wrist_1_joint", -2.829653},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_0_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.935973},
                {"floor_elbow_joint", 2.144842},
                {"floor_wrist_1_joint", -2.796700},
                {"floor_wrist_2_joint", -1.512684},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos25_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.770000},
                {"floor_elbow_joint", 2.020000},
                {"floor_wrist_1_joint", -2.815336},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos25_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.823280},
                {"floor_elbow_joint", 2.020000},
                {"floor_wrist_1_joint", -2.728673},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos25_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.908300},
                {"floor_elbow_joint", 2.050000},
                {"floor_wrist_1_joint", -2.7},
                {"floor_wrist_2_joint", -1.545000},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos50_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.757000},
                {"floor_elbow_joint", 1.990000},
                {"floor_wrist_1_joint", -2.815336},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos50_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.8},
                {"floor_elbow_joint", 1.967168},
                {"floor_wrist_1_joint", -2.728673},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos50_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.887000},
                {"floor_elbow_joint", 2.000176},
                {"floor_wrist_1_joint", -2.682060},
                {"floor_wrist_2_joint", -1.5},
                {"floor_wrist_3_joint", 0.0}};


            std::map<std::string, double> conv_pick_pos75_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.728000},
                {"floor_elbow_joint", 1.884336},
                {"floor_wrist_1_joint", 3.569318},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos75_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.774000},
                {"floor_elbow_joint", 1.901680},
                {"floor_wrist_1_joint", -2.704337},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos75_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.857336},
                {"floor_elbow_joint", 1.937344},
                {"floor_wrist_1_joint", -2.680000},
                {"floor_wrist_2_joint", -1.5},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos1_low_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.716682},
                {"floor_elbow_joint", 1.8717},
                {"floor_wrist_1_joint", -2.742943},
                {"floor_wrist_2_joint", -1.50},
                {"floor_wrist_3_joint", 0.0}};
            

            std::map<std::string, double> conv_pick_pos1_mid_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -0.735686},
                {"floor_elbow_joint", 1.764336},
                {"floor_wrist_1_joint", -2.504343},
                {"floor_wrist_2_joint", -1.51327},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_pos1_high_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.140000},
                {"floor_shoulder_lift_joint", -0.807780},
                {"floor_elbow_joint", 1.786921},
                {"floor_wrist_1_joint", 3.781281},
                {"floor_wrist_2_joint", -1.5},
                {"floor_wrist_3_joint", 0.0}};

            std::map<std::string, double> conv_pick_right_up_js = {
                {"linear_actuator_joint", -2.0},
                {"floor_shoulder_pan_joint", -3.14},
                {"floor_shoulder_lift_joint", -1.633},
                {"floor_elbow_joint", 1.507},
                {"floor_wrist_1_joint", -1.382},
                {"floor_wrist_2_joint", -1.45},
                {"floor_wrist_3_joint", 0.0}};


            // {
            //     offset :    -1  = 0.089995
            //     offset : -0.75  = 0.054995
            //     offset : -0.50  = 0.019997
            //     offset : -0.25  =-0.015004
            //     offset :     0  =-0.050003
            //     offset :  0.25  =-0.085005
            //     offset :  0.50  =-0.120006
            //     offset :  0.75  =-0.155004
            //     offset :     1  =-0.190005

            // }
            


            //#################################### BIN 1 #####################################
            std::map<std::string, double> bin_1_up = {
                        {"linear_actuator_joint", -3.8920472376049293},
                        {"floor_shoulder_pan_joint", 0.4895877470971639},
                        {"floor_shoulder_lift_joint",-0.6311989478668137},
                        {"floor_elbow_joint", 1.9216007358893261},
                        {"floor_wrist_1_joint", -2.8611957734417066},
                        {"floor_wrist_2_joint", -1.5707950783093036},
                        {"floor_wrist_3_joint", -1.0813310248453571}
                        };



            std::map<std::string, double> bin_1_slot_2_reg = {
                        {"linear_actuator_joint", -3.854666},
                        {"floor_shoulder_pan_joint", 0.359990},
                        {"floor_shoulder_lift_joint",-0.475000},
                        {"floor_elbow_joint",1.6},
                        {"floor_wrist_1_joint", -2.6731490},
                        {"floor_wrist_2_joint", -1.570795},
                        {"floor_wrist_3_joint", 0}
                        };




            std::map<std::string, double> bin_1_slot_4_reg = {
                        {"linear_actuator_joint", -3.739893},
                        {"floor_shoulder_pan_joint", 0.520740},
                        {"floor_shoulder_lift_joint",-0.535200},
                        {"floor_elbow_joint", 1.827820},
                        {"floor_wrist_1_joint", -2.848500},
                        {"floor_wrist_2_joint", -1.570790},
                        {"floor_wrist_3_joint", 0}
                        };


            std::map<std::string, double> bin_1_slot_6_reg = {
                        {"linear_actuator_joint", -4.04400},
                        {"floor_shoulder_pan_joint", 0.456892},
                        {"floor_shoulder_lift_joint",-0.552720},
                        {"floor_elbow_joint", 1.908010},
                        {"floor_wrist_1_joint", -2.908000},
                        {"floor_wrist_2_joint", -1.570795},
                        {"floor_wrist_3_joint", 0}
                        };



            std::map<std::string, double> bin_1_slot_8_reg = {
                        {"linear_actuator_joint", -3.890990},
                        {"floor_shoulder_pan_joint", 0.622700},
                        {"floor_shoulder_lift_joint",-0.576661},
                        {"floor_elbow_joint", 2.108418},
                        {"floor_wrist_1_joint", -3.095550},
                        {"floor_wrist_2_joint", -1.570790},
                        {"floor_wrist_3_joint", 0}
                        };


            std::map<std::string, double> bin_1_slot_1 = {
                        {"linear_actuator_joint", -3.7042374012241788},
                        {"floor_shoulder_pan_joint", 0.3903290011856923},
                        {"floor_shoulder_lift_joint",-0.5417662098020809},
                        {"floor_elbow_joint", 1.6199920874376383},
                        {"floor_wrist_1_joint", -2.6490197506839577},
                        {"floor_wrist_2_joint", -1.5707953168074655},
                        {"floor_wrist_3_joint", -1.1794221279005765}
                        };


            std::map<std::string, double> bin_1_slot_2 = {
                        {"linear_actuator_joint", -3.8546668348717694},
                        {"floor_shoulder_pan_joint", 0.3599937565141559},
                        {"floor_shoulder_lift_joint",-0.553959525081874},
                        {"floor_elbow_joint",1.656314749479431},
                        {"floor_wrist_1_joint", -2.673149067979141},
                        {"floor_wrist_2_joint", -1.5707953916182562},
                        {"floor_wrist_3_joint", -1.20972371405167}
                        };


            std::map<std::string, double> bin_1_slot_3 = {
                        {"linear_actuator_joint", -4.0043811790603545},
                        {"floor_shoulder_pan_joint", 0.3279971082716014},
                        {"floor_shoulder_lift_joint",-0.5653802138277574},
                        {"floor_elbow_joint", 1.6912706662583035},
                        {"floor_wrist_1_joint", -2.6966842673537235},
                        {"floor_wrist_2_joint", -1.57079547159924},
                        {"floor_wrist_3_joint", -1.241742224400602}
                        };


            std::map<std::string, double> bin_1_slot_4 = {
                        {"linear_actuator_joint", -3.739893079157342},
                        {"floor_shoulder_pan_joint", 0.5207458433532642},
                        {"floor_shoulder_lift_joint",-0.6217250706663922},
                        {"floor_elbow_joint", 1.8837826494525025},
                        {"floor_wrist_1_joint", -2.8328516041505027},
                        {"floor_wrist_2_joint", -1.570795006324359},
                        {"floor_wrist_3_joint", -1.0498350795530857}
                        };


            std::map<std::string, double> bin_1_slot_5 = {
                        {"linear_actuator_joint", -3.8920472376049293},
                        {"floor_shoulder_pan_joint", 0.4895877470971639},
                        {"floor_shoulder_lift_joint",-0.6311989478668137},
                        {"floor_elbow_joint", 1.9216007358893261},
                        {"floor_wrist_1_joint", -2.8611957734417066},
                        {"floor_wrist_2_joint", -1.5707950783093036},
                        {"floor_wrist_3_joint", -1.0813310248453571}
                        };


            std::map<std::string, double> bin_1_slot_6 = {
                        {"linear_actuator_joint", -4.044005812179439},
                        {"floor_shoulder_pan_joint", 0.4568927009325192},
                        {"floor_shoulder_lift_joint",-0.6397208388193588},
                        {"floor_elbow_joint", 1.958010738986193},
                        {"floor_wrist_1_joint", -2.8890838458231323},
                        {"floor_wrist_2_joint", -1.5707951558546946},
                        {"floor_wrist_3_joint", -1.1145564442093467}
                        };


            std::map<std::string, double> bin_1_slot_7 = {
                        {"linear_actuator_joint", -3.73674913899622},
                        {"floor_shoulder_pan_joint", 0.6544517467927243},
                        {"floor_shoulder_lift_joint",-0.6698775380722646},
                        {"floor_elbow_joint", 2.130483202322596},
                        {"floor_wrist_1_joint", -3.0313998859365765},
                        {"floor_wrist_2_joint", -1.5707947112901541},
                        {"floor_wrist_3_joint", -0.9173470390625171}
                        };


            std::map<std::string, double> bin_1_slot_8 = {
                        {"linear_actuator_joint", -3.890990795404263},
                        {"floor_shoulder_pan_joint", 0.6227554758166775},
                        {"floor_shoulder_lift_joint",-0.6736618855453961},
                        {"floor_elbow_joint", 2.168418706452916},
                        {"floor_wrist_1_joint", -3.0655509925913713},
                        {"floor_wrist_2_joint", -1.570794778639109},
                        {"floor_wrist_3_joint", -0.9489589753610989}
                        };


            std::map<std::string, double> bin_1_slot_9 = {
                        {"linear_actuator_joint", -4.045825840996818},
                        {"floor_shoulder_pan_joint", 0.5900404520412209},
                        {"floor_shoulder_lift_joint",-0.6760321348809218},
                        {"floor_elbow_joint", 2.2043279653793735},
                        {"floor_wrist_1_joint", -3.099089952742607},
                        {"floor_wrist_2_joint", -1.570794849907421},
                        {"floor_wrist_3_joint",-0.9816474296106212}
                        };

            //####################################### BIN 2 ################################

            std::map<std::string, double> bin_2_up = {
                        {"linear_actuator_joint", -3.2568942692863145},
                        {"floor_shoulder_pan_joint", 0.6100699088543328},
                        {"floor_shoulder_lift_joint",-0.836637},
                        {"floor_elbow_joint", 1.7548537938267585},
                        {"floor_wrist_1_joint", -2.488973},
                        {"floor_wrist_2_joint", -1.570794805898774},
                        {"floor_wrist_3_joint", -0.9595499400215338}};

            std::map<std::string, double> bin_2_slot_1 = {
                        {"linear_actuator_joint", -3.068763306289553},
                        {"floor_shoulder_pan_joint", 0.4992899072241625},
                        {"floor_shoulder_lift_joint",-0.483928239840804},
                        {"floor_elbow_joint", 1.4578554839190063},
                        {"floor_wrist_1_joint", -2.5447212414770877},
                        {"floor_wrist_2_joint", -1.570795055961034},
                        {"floor_wrist_3_joint", -1.0704703150957824}
                        };


            std::map<std::string, double> bin_2_slot_2 = {
                        {"linear_actuator_joint", -3.222593398452471},
                        {"floor_shoulder_pan_joint", 0.4755803191390767},
                        {"floor_shoulder_lift_joint",-0.4987462978547726},
                        {"floor_elbow_joint", 1.4980619680059792},
                        {"floor_wrist_1_joint", -2.570109638123152},
                        {"floor_wrist_2_joint", -1.5707951114661634},
                        {"floor_wrist_3_joint", -1.0941456977135071}
                        };


            std::map<std::string, double> bin_2_slot_3 = {
                        {"linear_actuator_joint", -3.375585328582454},
                        {"floor_shoulder_pan_joint", 0.45037817660777996},
                        {"floor_shoulder_lift_joint",-0.5130017840660502},
                        {"floor_elbow_joint", 1.537535188938739},
                        {"floor_wrist_1_joint", -2.5953273429514074},
                        {"floor_wrist_2_joint", -1.57079517134354},
                        {"floor_wrist_3_joint", -1.1193667796314688}
                        };


            std::map<std::string, double> bin_2_slot_4 = {
                        {"linear_actuator_joint", -3.103940847184637},
                        {"floor_shoulder_pan_joint", 0.635501006465444},
                        {"floor_shoulder_lift_joint",-0.5717982347417176},
                        {"floor_elbow_joint",1.7113441866869992},
                        {"floor_wrist_1_joint", -2.710340143465679},
                        {"floor_wrist_2_joint", -1.570794751386422},
                        {"floor_wrist_3_joint", -0.9342562029773271}
                        };


            std::map<std::string, double> bin_2_slot_5 = {
                        {"linear_actuator_joint", -3.2568942692863145},
                        {"floor_shoulder_pan_joint", 0.6100699088543328},
                        {"floor_shoulder_lift_joint",-0.5853106684669979},
                        {"floor_elbow_joint", 1.7548537938267585},
                        {"floor_wrist_1_joint", -2.740337277724708},
                        {"floor_wrist_2_joint", -1.570794805898774},
                        {"floor_wrist_3_joint", -0.9595499400215338}};


            std::map<std::string, double> bin_2_slot_6 = {
                        {"linear_actuator_joint", -3.409588740681816},
                        {"floor_shoulder_pan_joint", 0.5833007679211802},
                        {"floor_shoulder_lift_joint",-0.5978758974657316},
                        {"floor_elbow_joint",1.7971009146665367},
                        {"floor_wrist_1_joint", -2.770019129541897},
                        {"floor_wrist_2_joint", -1.5707948648016092},
                        {"floor_wrist_3_joint", -0.9863876815005631}
                        };


            std::map<std::string, double> bin_2_slot_7 = {
                        {"linear_actuator_joint", -3.0986298585018},
                        {"floor_shoulder_pan_joint", 0.7735956548149195},
                        {"floor_shoulder_lift_joint",-0.6383820812623432},
                        {"floor_elbow_joint", 1.952167189461559},
                        {"floor_wrist_1_joint", -2.884579536828078},
                        {"floor_wrist_2_joint", -1.570794472520664},
                        {"floor_wrist_3_joint", -0.7978927270489847}
                        };


            std::map<std::string, double> bin_2_slot_8 = {
                        {"linear_actuator_joint", -3.2512408906939583},
                        {"floor_shoulder_pan_joint", 0.7469682542356774},
                        {"floor_shoulder_lift_joint",-0.6482851994862444},
                        {"floor_elbow_joint", 1.9978630626614484},
                        {"floor_wrist_1_joint", -2.9203722432442376},
                        {"floor_wrist_2_joint", -1.5707945235523357},
                        {"floor_wrist_3_joint", -0.8247190228146454}
                        };


            std::map<std::string, double> bin_2_slot_9 = {
                        {"linear_actuator_joint", -3.404061126096928},
                        {"floor_shoulder_pan_joint", 0.7189949841043042},
                        {"floor_shoulder_lift_joint",-0.656720757408303},
                        {"floor_elbow_joint",2.0418894839386783},
                        {"floor_wrist_1_joint", -2.955963056957137},
                        {"floor_wrist_2_joint", -1.5707945786669866},
                        {"floor_wrist_3_joint", -0.8527050644781883}
                        };

            //###################################### BIN 3 ########################################


            std::map<std::string, double> bin_3_slot_7 = {
                        {"linear_actuator_joint", -2.797805716928974},
                        {"floor_shoulder_pan_joint", 0.14987733140318862},
                        {"floor_shoulder_lift_joint",-0.1824931372562651},
                        {"floor_elbow_joint", 0.732736649471225},
                        {"floor_wrist_1_joint", -2.1210372153722084},
                        {"floor_wrist_2_joint", -1.5707959303726169},
                        {"floor_wrist_3_joint", -1.4199221178529369}
                        };


            std::map<std::string, double> bin_3_slot_8 = {
                        {"linear_actuator_joint", -2.9582124754009493},
                        {"floor_shoulder_pan_joint", 0.13380371158746082},
                        {"floor_shoulder_lift_joint",-0.1938699851554617},
                        {"floor_elbow_joint", 0.758223597185395},
                        {"floor_wrist_1_joint", -2.135147309186723},
                        {"floor_wrist_2_joint", -1.5707959723814775},
                        {"floor_wrist_3_joint", -1.4359139295601993}
                        };


            std::map<std::string, double> bin_3_slot_9 = {
                        {"linear_actuator_joint", -3.117193206982079},
                        {"floor_shoulder_pan_joint", 0.1164411639188143},
                        {"floor_shoulder_lift_joint",-0.20499495820676425},
                        {"floor_elbow_joint",0.783252088927093},
                        {"floor_wrist_1_joint", -2.149050822130331},
                        {"floor_wrist_2_joint", -1.570796018037941},
                        {"floor_wrist_3_joint", -1.453255459650318}
                        };

            //######################################### BIN 4 #####################################


            std::map<std::string, double> bin_4_slot_7 = {
                        {"linear_actuator_joint", -3.4566126487079964},
                        {"floor_shoulder_pan_joint", 0.0742072560726745},
                        {"floor_shoulder_lift_joint",-0.22777056238002208},
                        {"floor_elbow_joint", 0.834769415800716},
                        {"floor_wrist_1_joint", -2.1777925341333004},
                        {"floor_wrist_2_joint", -1.5707961298616766},
                        {"floor_wrist_3_joint", -1.495591562899763}
                        };


            std::map<std::string, double> bin_4_slot_8 = {
                        {"linear_actuator_joint", -3.6108450646030557},
                        {"floor_shoulder_pan_joint",0.05247700933264704},
                        {"floor_shoulder_lift_joint",-0.23742543158732984},
                        {"floor_elbow_joint",0.8567446639750538},
                        {"floor_wrist_1_joint", -2.1901129094611584},
                        {"floor_wrist_2_joint", -1.5707961872049405},
                        {"floor_wrist_3_joint",-1.5172451582286222}
                        };


            std::map<std::string, double> bin_4_slot_9 = {
                        {"linear_actuator_joint", -3.7636693228969254},
                        {"floor_shoulder_pan_joint", 0.02943015751350703},
                        {"floor_shoulder_lift_joint",-0.24633225372642992},
                        {"floor_elbow_joint",0.8770729842337797},
                        {"floor_wrist_1_joint", -2.2015344050741876},
                        {"floor_wrist_2_joint", -1.5707962482490565},
                        {"floor_wrist_3_joint", -1.5402694121217406}
                        };



            //######################################### BIN 5 ############################################
            
            std::map<std::string, double> bin_5_up = {
                        {"linear_actuator_joint", 2.743078173121644},
                        {"floor_shoulder_pan_joint", 0.6100966719404742},
                        {"floor_shoulder_lift_joint",-0.836594},
                        {"floor_elbow_joint",1.7548054908252737},
                        {"floor_wrist_1_joint", -2.488992},
                        {"floor_wrist_2_joint", -1.5707948057277017},
                        {"floor_wrist_3_joint", -0.9594712708007602}
                        };
            
            std::map<std::string, double> pump_bin_5_slot_1 = {
                        {"linear_actuator_joint", 3.52000},
                        {"floor_shoulder_pan_joint", -0.185664},
                        {"floor_shoulder_lift_joint",-0.63960},
                        {"floor_elbow_joint", 1.907787},
                        {"floor_wrist_1_joint", -2.798602},
                        {"floor_wrist_2_joint", -1.573628},
                        {"floor_wrist_3_joint", -1.759296}
                        };

            std::map<std::string, double> bin_5_slot_1 = {
                        {"linear_actuator_joint", 2.931228683123155},
                        {"floor_shoulder_pan_joint", 0.4992970345864633},
                        {"floor_shoulder_lift_joint",-0.48391740798002},
                        {"floor_elbow_joint", 1.4578409074916658},
                        {"floor_wrist_1_joint", -2.544717496937238},
                        {"floor_wrist_2_joint", -1.5707950559120702},
                        {"floor_wrist_3_joint", -1.0704492966710017}
                        };

            std::map<std::string, double> bin_5_slot_2 = {
                        {"linear_actuator_joint", 2.777382150376331},
                        {"floor_shoulder_pan_joint", 0.4756027617232451},
                        {"floor_shoulder_lift_joint",-0.4987292647427746},
                        {"floor_elbow_joint", 1.4980248774486873},
                        {"floor_wrist_1_joint", -2.570089580739292},
                        {"floor_wrist_2_joint", -1.5707951113469034},
                        {"floor_wrist_3_joint", -1.094095142792559}
                        };

            std::map<std::string, double> bin_5_slot_3 = {
                        {"linear_actuator_joint", 2.6244344140834155},
                        {"floor_shoulder_pan_joint",0.4503597400209757},
                        {"floor_shoulder_lift_joint",-0.5130087624723737},
                        {"floor_elbow_joint", 1.5375621191875397},
                        {"floor_wrist_1_joint", -2.595347294822885},
                        {"floor_wrist_2_joint", -1.5707951712835706},
                        {"floor_wrist_3_joint", -1.119341675710097}
                        };           
            
            
            std::map<std::string, double> bin_5_slot_4 = {
                        {"linear_actuator_joint", 2.8960826090771103},
                        {"floor_shoulder_pan_joint", 0.6354795083430811},
                        {"floor_shoulder_lift_joint",-0.5718111079631627},
                        {"floor_elbow_joint", 1.7113847849055288},
                        {"floor_wrist_1_joint", -2.7103678685248447},
                        {"floor_wrist_2_joint", -1.5707947513022629},
                        {"floor_wrist_3_joint", -0.9342167925656418}
                        };
 
            std::map<std::string, double> bin_5_slot_5 = {
                        {"linear_actuator_joint", 2.743078173121644},
                        {"floor_shoulder_pan_joint", 0.6100966719404742},
                        {"floor_shoulder_lift_joint",-0.5852840037562037},
                        {"floor_elbow_joint",1.7548054908252737},
                        {"floor_wrist_1_joint", -2.7403156395536556},
                        {"floor_wrist_2_joint", -1.5707948057277017},
                        {"floor_wrist_3_joint", -0.9594712708007602}
                        };

            std::map<std::string, double> bin_5_slot_6 = {
                        {"linear_actuator_joint", 2.5903509611515427},
                        {"floor_shoulder_pan_joint", 0.5833611246712693},
                        {"floor_shoulder_lift_joint",-0.5978444369367281},
                        {"floor_elbow_joint", 1.7970100682752315},
                        {"floor_wrist_1_joint", -2.7699597438779335},
                        {"floor_wrist_2_joint", -1.5707948645011651},
                        {"floor_wrist_3_joint", -0.9862520139247224}
                        };

            std::map<std::string, double> bin_5_slot_7 = {
                        {"linear_actuator_joint", 2.9013072090425256},
                        {"floor_shoulder_pan_joint", 0.7736551508495356},
                        {"floor_shoulder_lift_joint",-0.6383588451311684},
                        {"floor_elbow_joint", 1.9520612176120686},
                        {"floor_wrist_1_joint",-2.8844968012159424},
                        {"floor_wrist_2_joint", -1.570794472413663},
                        {"floor_wrist_3_joint", -0.7978355470795784}
                        };

            std::map<std::string, double> bin_5_slot_8 = {
                        {"linear_actuator_joint", 2.748746779386358},
                        {"floor_shoulder_pan_joint", 0.7469806466748184},
                        {"floor_shoulder_lift_joint",-0.6482652965937499},
                        {"floor_elbow_joint", 1.9978382721838441},
                        {"floor_wrist_1_joint", -2.920367355729135},
                        {"floor_wrist_2_joint", -1.5707945234775536},
                        {"floor_wrist_3_joint", -0.8246806746354047}
                        };                       

            std::map<std::string, double> bin_5_slot_9 = {
                        {"linear_actuator_joint", 2.5959422617397125},
                        {"floor_shoulder_pan_joint", 0.718991251307449},
                        {"floor_shoulder_lift_joint",-0.65672481330857},
                        {"floor_elbow_joint", 2.0418946485701213},
                        {"floor_wrist_1_joint", -2.9559641655979725},
                        {"floor_wrist_2_joint", -1.5707945787701836},
                        {"floor_wrist_3_joint", -0.8527567504153601}
                        };


            // ####################################### BIN 6 ####################################

            std::map<std::string, double> bin_6_up = {
                        {"linear_actuator_joint", 2.1081064173594384},
                        {"floor_shoulder_pan_joint", 0.48941355062239644},
                        {"floor_shoulder_lift_joint",-1.003191},
                        {"floor_elbow_joint", 1.9200762818739157},
                        {"floor_wrist_1_joint", -2.487609},
                        {"floor_wrist_2_joint", -1.5707978873024633},
                        {"floor_wrist_3_joint", -1.0814829042734897}
                        };       
            
            std::map<std::string, double> bin_6_slot_1 = {
                        {"linear_actuator_joint", 2.296085400054},
                        {"floor_shoulder_pan_joint", 0.39000379196763724},
                        {"floor_shoulder_lift_joint",-0.5375716158266178},
                        {"floor_elbow_joint", 1.618775036729736},
                        {"floor_wrist_1_joint", -2.65200747373881},
                        {"floor_wrist_2_joint", -1.570797610785241},
                        {"floor_wrist_3_joint", -1.1796420919759696}
                        };


            std::map<std::string, double> bin_6_slot_2 = {
                        {"linear_actuator_joint", 2.145642470392843},
                        {"floor_shoulder_pan_joint", 0.35967270995462863},
                        {"floor_shoulder_lift_joint",-0.5496745262152466},
                        {"floor_elbow_joint", 1.655051754700936},
                        {"floor_wrist_1_joint", -2.6761807292401913},
                        {"floor_wrist_2_joint", -1.5707977411426044},
                        {"floor_wrist_3_joint",-1.209919177151432}
                        };

            std::map<std::string, double> bin_6_slot_3 = {
                        {"linear_actuator_joint", 1.9958375989446748},
                        {"floor_shoulder_pan_joint", 0.3277640310523931},
                        {"floor_shoulder_lift_joint",-0.5609902922831339},
                        {"floor_elbow_joint", 1.6898791497011143},
                        {"floor_wrist_1_joint", -2.6996917833821024},
                        {"floor_wrist_2_joint", -1.5707978759417096},
                        {"floor_wrist_3_joint",-1.2418791651701155}
                        };

            std::map<std::string, double> bin_6_slot_4 = {
                        {"linear_actuator_joint", 2.2602058472351523},
                        {"floor_shoulder_pan_joint", 0.5206386975759858},
                        {"floor_shoulder_lift_joint",-0.6168416450438783},
                        {"floor_elbow_joint", 1.8822182634793094},
                        {"floor_wrist_1_joint", -2.836175957403545},
                        {"floor_wrist_2_joint", -1.5707977438626695},
                        {"floor_wrist_3_joint", -1.0497953717885584}
                        };

            std::map<std::string, double> bin_6_slot_5 = {
                        {"linear_actuator_joint", 2.1081064173594384},
                        {"floor_shoulder_pan_joint", 0.48941355062239644},
                        {"floor_shoulder_lift_joint",-0.626220959575509},
                        {"floor_elbow_joint", 1.9200762818739157},
                        {"floor_wrist_1_joint", -2.864653717774771},
                        {"floor_wrist_2_joint", -1.5707978873024633},
                        {"floor_wrist_3_joint", -1.0814829042734897}
                        };

            std::map<std::string, double> bin_6_slot_6 = {
                        {"linear_actuator_joint", 1.9560106946859017},
                        {"floor_shoulder_pan_joint", 0.45687446103088547},
                        {"floor_shoulder_lift_joint",-0.6345820045429635},
                        {"floor_elbow_joint", 1.9562786013149942},
                        {"floor_wrist_1_joint", -2.8924940424240897},
                        {"floor_wrist_2_joint", -1.5707980330945204},
                        {"floor_wrist_3_joint", -1.11448372980044}
                        };

            std::map<std::string, double> bin_6_slot_7 = {
                        {"linear_actuator_joint", 2.2630020184816377},
                        {"floor_shoulder_pan_joint", 0.6547515660917532},
                        {"floor_shoulder_lift_joint",-0.6640772446299114},
                        {"floor_elbow_joint", 2.1281980638707783},
                        {"floor_wrist_1_joint", -3.034913657935872},
                        {"floor_wrist_2_joint", -1.570797913616046},
                        {"floor_wrist_3_joint", -0.9171176976069013}
                        };          
            
            std::map<std::string, double> bin_6_slot_8 = {
                        {"linear_actuator_joint", 2.108756675753841},
                        {"floor_shoulder_pan_joint", 0.6230769571975581},
                        {"floor_shoulder_lift_joint",-0.6677075660729481},
                        {"floor_elbow_joint",2.1660892089805883},
                        {"floor_wrist_1_joint", -3.0691732537143492},
                        {"floor_wrist_2_joint", -1.570798046703739},
                        {"floor_wrist_3_joint", -0.948739166490242}
                        };


            std::map<std::string, double> bin_6_slot_9 = {
                        {"linear_actuator_joint", 2.108756675753841},
                        {"floor_shoulder_pan_joint", 0.5904410235525892},
                        {"floor_shoulder_lift_joint",-0.6699321050910012},
                        {"floor_elbow_joint", 2.201898269848624},
                        {"floor_wrist_1_joint", -3.1027565920055102},
                        {"floor_wrist_2_joint", -1.5707981751385074},
                        {"floor_wrist_3_joint", -0.9813326406687246}
                        };

            
            //####################################### BIN 7 ################################

            std::map<std::string, double> bin_7_slot_7 = {
                        {"linear_actuator_joint", 2.5433676187079697},
                        {"floor_shoulder_pan_joint", 0.0742238250351846},
                        {"floor_shoulder_lift_joint",-0.2277538641371265},
                        {"floor_elbow_joint", 0.8347466343557606},
                        {"floor_wrist_1_joint", -2.177786450934225},
                        {"floor_wrist_2_joint", -1.5707961298215298},
                        {"floor_wrist_3_joint",-1.4955763917667044}
                        };

            std::map<std::string, double> bin_7_slot_8 = {
                        {"linear_actuator_joint", 2.389085503396457},
                        {"floor_shoulder_pan_joint", 0.052535577955578845},
                        {"floor_shoulder_lift_joint",-0.23740023586285575},
                        {"floor_elbow_joint", 0.8566894387531724},
                        {"floor_wrist_1_joint", -2.1900828799744376},
                        {"floor_wrist_2_joint", -1.5707961870018983},
                        {"floor_wrist_3_joint", -1.5171685365204017}
                        };

            std::map<std::string, double> bin_7_slot_9 = {
                        {"linear_actuator_joint", 2.2363254786161937},
                        {"floor_shoulder_pan_joint", 0.029434323844932893},
                        {"floor_shoulder_lift_joint",-0.24633272160430972},
                        {"floor_elbow_joint", 0.8770721943233626},
                        {"floor_wrist_1_joint", -2.201533147289147},
                        {"floor_wrist_2_joint", -1.5707962481388675},
                        {"floor_wrist_3_joint", -1.5402278693075477}
                        };

            //##################################### BIN 8 ##################################

            std::map<std::string, double> bin_8_slot_7 = {
                        {"linear_actuator_joint", 3.2021982894885253},
                        {"floor_shoulder_pan_joint", 0.14987405203532495},
                        {"floor_shoulder_lift_joint",-0.18249754536607565},
                        {"floor_elbow_joint", 0.7327433916026557},
                        {"floor_wrist_1_joint",-2.1210395493922944},
                        {"floor_wrist_2_joint", -1.5707959303827659},
                        {"floor_wrist_3_joint", -1.4199259849759116}
                        };

            std::map<std::string, double> bin_8_slot_8 = {
                        {"linear_actuator_joint",3.041767632641657},
                        {"floor_shoulder_pan_joint", 0.13382000899554403},
                        {"floor_shoulder_lift_joint",-0.19385622819450937},
                        {"floor_elbow_joint", 0.7581974552321264},
                        {"floor_wrist_1_joint",-2.135134924202327},
                        {"floor_wrist_2_joint", -1.5707959723226796},
                        {"floor_wrist_3_joint", -1.4358915706964364}
                        };

            std::map<std::string, double> bin_8_slot_9 = {
                        {"linear_actuator_joint", 2.882787258811323},
                        {"floor_shoulder_pan_joint", 0.11645727396576988},
                        {"floor_shoulder_lift_joint",-0.20498330878954313},
                        {"floor_elbow_joint", 0.78322959405237},
                        {"floor_wrist_1_joint", -2.149039976682256},
                        {"floor_wrist_2_joint", -1.5707960179574454},
                        {"floor_wrist_3_joint", -1.453224917682339}
                        };

            std::map<std::string, double> drop_faulty_part = {
                        {"linear_actuator_joint", 0.0000},
                        {"floor_shoulder_pan_joint", -0.251327},
                        {"floor_shoulder_lift_joint",-0.879646},
                        {"floor_elbow_joint", 1.759292},
                        {"floor_wrist_1_joint", -2.513274},
                        {"floor_wrist_2_joint", -1.759292},
                        {"floor_wrist_3_joint", 2.010619}
                        };

            std::map<std::string, double> floor_flipped_js_ = {
                        {"linear_actuator_joint", -0.096000},
                        {"floor_shoulder_pan_joint", 0.0},
                        {"floor_shoulder_lift_joint",-0.628278},
                        {"floor_elbow_joint", 0.480},
                        {"floor_wrist_1_joint", -1.4},
                        {"floor_wrist_2_joint", -4.775215},
                        {"floor_wrist_3_joint", 0.0}
                        };
            
            std::map<std::string, double> floor_flipped_js_up = {
                        {"linear_actuator_joint", -0.14000},
                        {"floor_shoulder_pan_joint", 0.0},
                        {"floor_shoulder_lift_joint",-1},
                        {"floor_elbow_joint", 0.063},
                        {"floor_wrist_1_joint", -0.699},
                        {"floor_wrist_2_joint", -1.6},
                        {"floor_wrist_3_joint", 0.0}
                        };
                
            std::map<std::string, double> floor_straight_js_ = {
                        {"linear_actuator_joint", 0.0},
                        {"floor_shoulder_pan_joint", -1.57},
                        {"floor_shoulder_lift_joint",0.0},
                        {"floor_elbow_joint", 0.0},
                        {"floor_wrist_1_joint", 0.0},
                        {"floor_wrist_2_joint", 0.0},
                        {"floor_wrist_3_joint", 0.0}
                        };


            // ############################ ASSEMBLY ##################################
              std::map<std::string, double> ceiling_as1_js_ = {
                {"gantry_x_axis_joint", 1},
                {"gantry_y_axis_joint", -3},
                {"gantry_rotation_joint", 1.571},
                {"ceiling_shoulder_pan_joint", 0},
                {"ceiling_shoulder_lift_joint", -2.37},
                {"ceiling_elbow_joint", 2.37},
                {"ceiling_wrist_1_joint", 3.14},
                {"ceiling_wrist_2_joint", -1.57},
                {"ceiling_wrist_3_joint", 0}
                };

            std::map<std::string, double> ceiling_as2_js_ = {
                {"gantry_x_axis_joint", -4},
                {"gantry_y_axis_joint", -3},
                {"gantry_rotation_joint", 1.571},
                {"ceiling_shoulder_pan_joint", 0},
                {"ceiling_shoulder_lift_joint", -2.37},
                {"ceiling_elbow_joint", 2.37},
                {"ceiling_wrist_1_joint", 3.14},
                {"ceiling_wrist_2_joint", -1.57},
                {"ceiling_wrist_3_joint", 0}
                };

            std::map<std::string, double> ceiling_as3_js_ = {
                {"gantry_x_axis_joint", 1},
                {"gantry_y_axis_joint", 3},
                {"gantry_rotation_joint", 1.571},
                {"ceiling_shoulder_pan_joint", 0},
                {"ceiling_shoulder_lift_joint", -2.37},
                {"ceiling_elbow_joint", 2.37},
                {"ceiling_wrist_1_joint", 3.14},
                {"ceiling_wrist_2_joint", -1.57},
                {"ceiling_wrist_3_joint", 0}
                };

            std::map<std::string, double> ceiling_as4_js_ = {
                {"gantry_x_axis_joint", -4},
                {"gantry_y_axis_joint", 3},
                {"gantry_rotation_joint", 1.571},
                {"ceiling_shoulder_pan_joint", 0},
                {"ceiling_shoulder_lift_joint", -2.37},
                {"ceiling_elbow_joint", 2.37},
                {"ceiling_wrist_1_joint", 3.14},
                {"ceiling_wrist_2_joint", -1.57},
                {"ceiling_wrist_3_joint", 0}
                };           

            std::map<std::string, double> ceiling_flipped_js_up = {
                {"gantry_x_axis_joint", 3.18},
                {"gantry_y_axis_joint", 0.78},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", -1.904926},
                {"ceiling_shoulder_lift_joint", 0.0},
                {"ceiling_elbow_joint", 0.0},
                {"ceiling_wrist_1_joint", -1.632664},
                {"ceiling_wrist_2_joint", 0},
                {"ceiling_wrist_3_joint", 0}
                };   
            
            std::map<std::string, double> ceiling_flipped_js_ = {
                {"gantry_x_axis_joint", 3.18},
                {"gantry_y_axis_joint", 0.78},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", -1.653599},
                {"ceiling_shoulder_lift_joint", 0.0},
                {"ceiling_elbow_joint", 0.0},
                {"ceiling_wrist_1_joint", -1.632664},
                {"ceiling_wrist_2_joint", 0},
                {"ceiling_wrist_3_joint", 0}
                };  

            std::map<std::string, double> ceiling_flipped_js_down = {
                {"gantry_x_axis_joint", 3.6},
                {"gantry_y_axis_joint", 0.7},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", -1.508},
                {"ceiling_shoulder_lift_joint", 0.0},
                {"ceiling_elbow_joint", 0.0},
                {"ceiling_wrist_1_joint", 4.6495},
                {"ceiling_wrist_2_joint", 3.14},
                {"ceiling_wrist_3_joint", 0}
                };  

            std::map<std::string, double> ceiling_flipped_js_down1 = {
                {"gantry_x_axis_joint", 3.6},
                {"gantry_y_axis_joint", 0.7},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", -1.508},
                {"ceiling_shoulder_lift_joint", 0.0},
                {"ceiling_elbow_joint", 0.062833},
                {"ceiling_wrist_1_joint", 4.523837},
                {"ceiling_wrist_2_joint", 6.281592},
                {"ceiling_wrist_3_joint", 0}
                }; 

            std::map<std::string, double> ceiling_flipped_js_down2 = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", -3.47},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.880944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", 6.281592},
                {"ceiling_wrist_3_joint", 0}
                }; 

            std::map<std::string, double> ceiling_agv1_js_drop = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", -3.47},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0.0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.130944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", -1.6336},
                {"ceiling_wrist_3_joint", 0}
                };    

            std::map<std::string, double> ceiling_agv2_js_up = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", 0.12},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0.0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.880944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", -1.6336},
                {"ceiling_wrist_3_joint", 0}
                }; 

            std::map<std::string, double> ceiling_agv2_js_drop = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", 0.12},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0.0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.130944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", -1.6336},
                {"ceiling_wrist_3_joint", 0}
                }; 
            

            std::map<std::string, double> ceiling_agv3_js_drop = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", 2.55},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0.0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.130944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", -1.6336},
                {"ceiling_wrist_3_joint", 0}
                }; 

            std::map<std::string, double> ceiling_agv4_js_drop = {
                {"gantry_x_axis_joint", 4.95},
                {"gantry_y_axis_joint", 6.15},
                {"gantry_rotation_joint", 0.0},
                {"ceiling_shoulder_pan_joint", 0.0},
                {"ceiling_shoulder_lift_joint", 0.125652},
                {"ceiling_elbow_joint", -1.130944},
                {"ceiling_wrist_1_joint", -2.136391},
                {"ceiling_wrist_2_joint", -1.6336},
                {"ceiling_wrist_3_joint", 0}
                };       

    };
}
