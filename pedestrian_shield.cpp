//
// Created by fcano on 29.07.21.
//

#include <iostream>
#include "delayedShield.cpp"
#include <vector>
#include <string>
using namespace std;

int MIN_VEL = 0;
int MAX_VEL = 20;
int MIN_DIST = 0;
int MAX_DIST = 100;
int SAFETY_DIST = 3; // distance from where things are unsafe, in m
int SAFETY_VEL = 2; // speed from where things are unsafe, in m/s
const float timestep = 0.5; // delta t, in seconds
const int ego_speed_acc = 1; // changed from 2
const int brake_acc = -2; // changed from -1
const int vel_wait_to_brake = 8;

string get_state(int pos1, int vel, int pos2, bool key, bool wait) {
    string s = key ? "c" : "e"; //key car(c) or envirnoment(e)
    string sepchar = "_";
    s = s + sepchar + to_string(pos1) + sepchar + to_string(vel) + sepchar + to_string(pos2);
    if (vel <= vel_wait_to_brake) {
        string aux = wait ? "y" : "n";
        s = s + sepchar+ aux;
    }
    return s;
}

void get_values_from_state(string state, int& dist1, int& vel, int& dist2, bool& key, bool& wait) {
    assert(state[0] == 'c' or state[0] == 'e');
    string sepchar = "_";
    key = state[0] == 'c';
    int pos = 0;
    int prevpos = 1;
    for (int i = 0; i < 3; ++ i) {
        pos = state.find(sepchar, prevpos+1);
        if (i == 0)
            dist1 = stoi(state.substr(prevpos+1, pos - prevpos));
        if (i == 1)
            vel = stoi(state.substr(prevpos+1, pos - prevpos));
        if (i == 2)
            dist2 = stoi(state.substr(prevpos+1, pos - prevpos));
        prevpos = pos;
    }
    wait = false;
    if (vel <= vel_wait_to_brake) {
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
    for (int dist1 = MIN_DIST; dist1 <= MAX_DIST; ++dist1) {
        for (int vel = MIN_VEL; vel <= MAX_VEL; ++vel) {
            for (int dist2 = MIN_DIST; dist2 <= MAX_DIST; ++dist2) {
                for (int i = 0; i < 2; ++i) {
                    string state = get_state(dist1,vel,dist2,false, i == 1);
                    stateMap[state] = statecount++;
                    stateNames.push_back(state);
                    state = get_state(dist1,vel,dist2,true, i == 1);
                    stateMap[state] = statecount++;
                    stateNames.push_back(state);
                    if (vel > vel_wait_to_brake) i = 2;
                }
            }
        }
    }

    // add safestate for both ego and env
    string safestate = get_state(-10,0,0,false,false);
    stateMap[safestate] = statecount++;
    stateNames.push_back(safestate);
    safestate = get_state(-10,0,0,true,false);
    stateMap[safestate] = statecount++;
    stateNames.push_back(safestate);
    // safestate done

    int numOfStates = stateNames.size();
    vector<pair<VI,VI>> Graph(numOfStates);
    for (int i = 0; i < labelNames.size(); ++i)
        labelNums[labelNames[i]] = i;

    for (int i = 0; i < numOfStates; ++i) {
        if (i%100000 == 0) cout << "We start state number " << i << endl;
        string state = stateNames[i];
        int dist1, vel, dist2;
        bool key, wait;
        get_values_from_state(state, dist1, vel, dist2, key,wait);
        VI outedges(0);
        if (key) { // ego turn
            int plus_speed;
            for (string action : labelNames) {
                // I have no idea wether these values make sense. Need to make simulations --> Palmi
                if (action == "U") continue;
                if (action == "D") plus_speed = brake_acc; // changed from "wait ? 0 : brake_acc;" 
                if (action == "S") plus_speed = 0; // changed from wait ? -brake_acc : 0;
                if (action == "A") plus_speed = ego_speed_acc;
                int vel_new = vel + plus_speed;
                if (vel_new > MAX_VEL) vel_new = MAX_VEL;
                if (vel_new < MIN_VEL) vel_new = MIN_VEL;
                int dist1_new = dist1 - vel_new*timestep;
                if (dist1_new > MAX_DIST) dist1_new = MAX_DIST;
                // if (dist1_new < MIN_DIST) dist1_new = MIN_DIST;
                bool new_wait = false;
                if (action == "A" and vel_new <= vel_wait_to_brake) new_wait = true;

                string out_state = get_state(dist1_new, vel_new, dist2, false, new_wait);
                if (dist1_new < MIN_DIST) {
                    out_state = get_state(-10,0,0, false, false);
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
            for (int j = -1; j <= 1; ++j) {
                for (int k = -1; k <= 1; ++k) {
                    if (j*j + k*k < 2) {
                        int dist2_new = dist2 + j;
                        int dist1_new = dist1 +k;
                        if (dist1_new > MAX_DIST) dist1_new = MAX_DIST;
                        // if (dist1_new < MIN_DIST) dist1_new = MIN_DIST;
                        if (dist2_new > MAX_DIST) dist2_new = MAX_DIST;
                        if (dist2_new < MIN_DIST) dist2_new = MIN_DIST;
                        string out_state = get_state(dist1_new,vel,dist2_new,true, wait);
                        if (dist1_new < MIN_DIST) {
                            out_state = get_state(-10,0,0, true, false);
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
        int dist1, dist2, vel;
        bool key, wait;
        get_values_from_state(stateNames[i], dist1, vel, dist2, key, wait);
        Ustates[i] = (vel >= SAFETY_VEL) and (dist1 <= SAFETY_DIST) and (dist1 >= dist2); // this does not contain safestate becuase dist1 < 0
    }
    int s0 = stateMap[get_state(MAX_DIST, MIN_VEL, MAX_DIST, true, false)]; // provisional as init state
    cout << "s0 is :" << stateNames[s0] << ", " << s0 << endl;

    delayedShield shield = delayedShield();
    shield.initialize_with_game(numOfStates, numOfStrategies, Delta, s0,
                                compute_safety_level, compute_controllability_level, compute_memoryless,
                                stateNames, labelNames, labelNums,
                                Graph, EdgeLabel, stateMap, Ustates, S0states, S1states);
    shield.run();
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
        keyword = "SAFETY_VEL=";
        if (argument.find(keyword) != std::string::npos)
            SAFETY_VEL = stoi(argument.substr(keyword.size(), argument.size() - keyword.size()));
    }
    delayedShield shield = generateShieldObject(max_delta, play_safety_strategy, play_controllability_strategy, compute_memoryless_strategy);
//    shield.printGraph();
    shield.printStrategy();

}
