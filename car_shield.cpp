//
// Created by fcano on 29.07.21.
//

//
// Created by fcano on 29.07.21.
//

#include <iostream>
#include "delayedShield.cpp"
#include <vector>
#include <string>
#include <cmath>
using namespace std;

int MIN_VEL = 0;
int MAX_VEL = 10;
int MIN_DIST = 0;
int MAX_DIST = 100;
int SAFETY_DIST = 2; // distance from where things are unsafe, in m [car lenght, more or less]
const float timestep = 0.5; // delta t, in seconds
const int ego_speed_acc = 1; // changed from 2
const int adv_speed_acc = 1; // changed from 2
const int brake_acc = -2; // changed from -1
const int vel_wait_to_brake = 8;
const int passing_pos = -1;


int round2to0(int x) {
    if (x > 0) return x%2 == 0 ? x : x-1;
    else return x%2 == 0 ? x : x+1;
}

int round2to0(float x) {
    int y = x > 0 ? int(floor(x)) : int(ceil(x));
    return round2to0(y);
}

int round2away0(int x) {
    if (x > 0) return x%2 == 0 ? x : x+1;
    else return x%2 == 0 ? x : x-1;
}

int round2away0(float x) {
    int y = x > 0 ? int(ceil(x)) : int(floor(x));
    return round2away0(y);
}


string get_state(int ego_pos, int ego_vel, int adv_pos, int adv_vel, bool key, bool wait) {
    string s = key ? "c" : "e"; //key car(c) or envirnoment(e)
    string sepchar = "_";
    s = s + sepchar + to_string(ego_pos) + sepchar + to_string(ego_vel);
    s = s + sepchar + to_string(adv_pos) + sepchar + to_string(adv_vel);
    if (ego_vel <= vel_wait_to_brake) {
        string aux = wait ? "y" : "n";
        s = s + sepchar+ aux;
    }
    return s;
}

void check_state_names(const map<string,int>& stateMap, const vector<string>& stateNames) {
    for (int i = 0; i < stateNames.size(); ++i) {
        assert(stateMap.at(stateNames[i]) == i);
    }
}

void get_values_from_state(string state, int& ego_dist, int& ego_vel, int& adv_dist, int& adv_vel, bool& key, bool& wait) {
    assert(state[0] == 'c' or state[0] == 'e');
    string sepchar = "_";
    key = state[0] == 'c';
    int pos = 0;
    int prevpos = 1;
    for (int i = 0; i < 4; ++ i) {
        pos = state.find(sepchar, prevpos+1);
        if (i == 0)
            ego_dist = stoi(state.substr(prevpos+1, pos - prevpos));
        if (i == 1)
            ego_vel = stoi(state.substr(prevpos+1, pos - prevpos));
        if (i == 2)
            adv_dist = stoi(state.substr(prevpos+1, pos - prevpos));
        if (i == 3)
            adv_vel = stoi(state.substr(prevpos+1, pos - prevpos));
        prevpos = pos;
    }
    wait = false;
    if (ego_vel <= vel_wait_to_brake) {
        wait = state[state.size()-1] == 'y';
    }
}

delayedShield generateShieldObject(int Delta, bool compute_safety_level, bool compute_controllability_level, bool compute_memoryless) {
    map<string, int> stateMap; // numbers of states
    vector<string> stateNames(0); // names of states
    map<pair<int,int>, VI> EdgeLabel;
    vector<string> labelNames = {"U", "D", "S", "A"}; // Undetermined (env), Decelerate, Stay, Accelerate
    int numOfStrategies = 4;
    map<string,int> labelNums;
    vector<bool> Ustates, S0states, S1states; //dont know yet about initialization
    int statecount = 0;
    for (int ego_dist = MIN_DIST; ego_dist <= MAX_DIST; ego_dist +=2) {
        for (int ego_vel = MIN_VEL; ego_vel <= MAX_VEL; ego_vel +=1) {
            for (int adv_dist = MIN_DIST; adv_dist <= MAX_DIST; adv_dist += 2) {
                for (int adv_vel = MIN_VEL; adv_vel <= MAX_VEL; adv_vel += 1) {
                    for (int i = 0; i < 2; ++i) {
                        string state = get_state(ego_dist,ego_vel,adv_dist, adv_vel,false, i ==0);
                        stateMap[state] = statecount++;
                        stateNames.push_back(state);
                        state = get_state(ego_dist,ego_vel,adv_dist, adv_vel,true, i == 0);
                        stateMap[state] = statecount++;
                        stateNames.push_back(state);
                        if (ego_vel > vel_wait_to_brake) i = 2;
                    }
                }
            }
        }
    }
    // add safestate for both ego and env
    string safestate = get_state(-10,0,-10,0,false,false);
    stateMap[safestate] = statecount++;
    stateNames.push_back(safestate);
    safestate = get_state(-10,0,-10,0,true,false);
    stateMap[safestate] = statecount++;
    stateNames.push_back(safestate);
    // safestate done

    int numOfStates = stateNames.size();
    vector<pair<VI,VI>> Graph(numOfStates);
    for (int i = 0; i < labelNames.size(); ++i)
        labelNums[labelNames[i]] = i;
    cout << "Start generation of transitions\n";
    for (int i = 0; i < numOfStates; ++i) {
        if (i%100000 == 0) cout << "We start state number " << i << endl;
        string state = stateNames[i];
        int ego_dist, ego_vel, adv_dist, adv_vel;
        bool key, wait;
        get_values_from_state(state, ego_dist, ego_vel, adv_dist, adv_vel, key, wait);
        VI outedges(0);
        if (key) { // ego turn
            int plus_speed;
            for (string action : labelNames) {
                if (action == "U") continue;
                if (action == "D") plus_speed = brake_acc; // changed from "wait ? 0 : brake_acc;" 
                if (action == "S") plus_speed = 0; // changed from wait ? -brake_acc : 0;
                if (action == "A") plus_speed = ego_speed_acc;
                int vel_new = ego_vel + plus_speed;
                if (vel_new > MAX_VEL) vel_new = MAX_VEL;
                if (vel_new < MIN_VEL) vel_new = MIN_VEL;
                int dist_new = round2to0(ego_dist - vel_new*timestep);
                if (dist_new > MAX_DIST) dist_new = MAX_DIST;
                // if (dist_new < MIN_DIST) dist_new = MIN_DIST; // changed from MIN_DIST, supposed to make going less than min dist safe again
                bool new_wait = false;
                if (action == "A" and ego_vel <= vel_wait_to_brake) new_wait = true;

                string out_state = get_state(dist_new, vel_new, adv_dist, adv_vel, false, new_wait);
                if (dist_new < MIN_DIST) {
                    out_state = get_state(-10, 0, -10, 0, false, false);
                }
                int outstate = stateMap[out_state];
                outedges.push_back(outstate);
                if (EdgeLabel.find({i, outstate}) == EdgeLabel.end()) {
                    EdgeLabel[{i,outstate}] = VI(1,labelNums[action]);
                }
                else
                    EdgeLabel[{i,outstate}].push_back(labelNums[action]);
            }
        }
        else { // adv turn
            int plus_speed;
            for (string action : labelNames) {
                // I have no idea wether these values make sense. Need to make simulations --> Palmi
                if (action == "U") continue;
                if (action == "D") plus_speed = brake_acc;
                if (action == "S") plus_speed = 0;
                if (action == "A") plus_speed =  adv_speed_acc;
                int vel_new = adv_vel + plus_speed;
                if (vel_new > MAX_VEL) vel_new = MAX_VEL;
                if (vel_new < MIN_VEL) vel_new = MIN_VEL;
                int dist_new = round2to0(adv_dist - vel_new*timestep);
                if (dist_new > MAX_DIST) dist_new = MAX_DIST;
                if (dist_new < MIN_DIST) dist_new = -10; // changed from MIN_DIST, supposed to make going less than min dist safe again

                string out_state = get_state(ego_dist, ego_vel,dist_new, vel_new, true, wait);
                if (dist_new < MIN_DIST) {
                    out_state = get_state(-10, 0, -10, 0, true, false);
                }
                int outstate = stateMap[out_state];
                outedges.push_back(outstate);
                if (EdgeLabel.find({i, outstate}) == EdgeLabel.end()) {
                    EdgeLabel[{i,outstate}] = VI(1,labelNums["U"]);
                }
                else
                    EdgeLabel[{i,outstate}].push_back(labelNums["U"]);
            }
        }
        VI inedges(0);
        Graph[i] = {inedges, outedges};
    }
    for (int i = 0; i < stateNames.size(); ++i) {
        for (int j : Graph[i].second)
            Graph[j].first.push_back(i);
    }
    for (int i = 0; i < Graph.size(); ++i) {
        delayedShield::remove_duplicates(Graph[i].first);
        delayedShield::remove_duplicates(Graph[i].second);
    }
    S0states = S1states = Ustates = vector<bool>(numOfStates, false);
    for (int i = 0; i < numOfStates; ++i) {
        S0states[i] = stateNames[i][0] == 'c';
        S1states[i] = !S0states[i];
    }
    for (int i = 0; i < numOfStates; ++i) {
        int ego_dist, ego_vel, adv_dist, adv_vel;
        bool key, wait;
        get_values_from_state(stateNames[i], ego_dist, ego_vel, adv_dist, adv_vel, key, wait);
        Ustates[i] = (ego_dist <= SAFETY_DIST) and (adv_dist <= SAFETY_DIST) and (ego_dist >= -4) and (adv_dist >= -4); // this should somehow be safety_Dist
    }
    auto aux = get_state(MAX_DIST, MIN_VEL, MAX_DIST, MIN_VEL, true, false);
    int s0 = stateMap[get_state(MAX_DIST, MIN_VEL, MAX_DIST, MIN_VEL, true, false)]; // provisional as init state
    cout << "s0 is :" << stateNames[s0] << ", " << s0 << endl;

    check_state_names(stateMap, stateNames);

    delayedShield shield = delayedShield();
    shield.initialize_with_game(numOfStates, numOfStrategies, Delta, s0,
                                compute_safety_level, compute_controllability_level, compute_memoryless,
                                stateNames, labelNames, labelNums,
                                Graph, EdgeLabel, stateMap, Ustates, S0states, S1states);
    return shield;
}


int main(int argc,char **argv) {
    bool play_safety_strategy = false;
    bool play_controllability_strategy = false;
    bool compute_memoryless_strategy = false;
    bool generate_graph_for_mathematica = false;
    int max_delta = 6;
    for (int i = 0; i < argc; ++i) {
        string argument = argv[i];
        if (argument == "play_controllable") play_controllability_strategy = true;
        if (argument == "play_safest") play_safety_strategy = true;
        if (argument == "compute_memoryless") compute_memoryless_strategy = true;
        if (argument == "mathematica") generate_graph_for_mathematica = true;
        string keyword = "delta=";
        if (argument.find(keyword) != std::string::npos)
            max_delta = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
        keyword = "MIN_VEL=";
        if (argument.find(keyword) != std::string::npos)
            MIN_VEL = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
        keyword = "MAX_VEL=";
        if (argument.find(keyword) != std::string::npos)
            MAX_VEL = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
        keyword = "MIN_DIST=";
        if (argument.find(keyword) != std::string::npos)
            MIN_DIST = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
        keyword = "MAX_DIST=";
        if (argument.find(keyword) != std::string::npos)
            MAX_DIST = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
        keyword = "SAFETY_DIST=";
        if (argument.find(keyword) != std::string::npos)
            SAFETY_DIST = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
    }
  
    delayedShield shield = generateShieldObject(max_delta, play_safety_strategy, play_controllability_strategy, compute_memoryless_strategy);
    shield.run();
//    shield.printGraph();
//    cout << endl << endl;
    shield.printStrategy();
}
