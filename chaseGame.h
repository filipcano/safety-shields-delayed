#pragma once

#include <iostream>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <utility>
#include <queue>
#include <stack>
//#include "delayedShield.cpp"
using namespace std;

class chaseGame {
    int M, N; // M = max(x) , N = max(y)
    bool key; //  key = true --> robot moves, false --> kid moves
    bool play_controllability_strategy;
    bool play_safety_strategy;
    bool compute_memoryless; // only computes it, chase game playing memoryless is not implemented
    pair<int, int> pos_robot, pos_kid;
    vector<pair<int,int> > hist_pos_robot, hist_pos_kid;
    map<char, pair<int,int>> basic_moves;
    vector<string> robot_moves, kid_moves; // (x,y) robot, (x,y) kid,
    vector<int> hist_robot_moves;
    vector<pair<int,int>> Obstacles;
    vector<pair<int,int>> Rewards;
    vector<string> debug_history_states;
    vector<string> debug_robot_moves;
    vector<pair<int,int>> non_obstacled_locations;
    vector<vector<string>> best_move_towards_reward;
    vector<vector<int>> best_dist_towards_reward;

    string pos2state(pair<int,int> robot, pair<int,int> kid, bool tempkey);

    void state2pos(string state, pair<int,int>& robot, pair<int,int>& kid, bool& tempkey);

    static pair<int, int> sum(pair<int,int> a, pair<int, int> b);

    pair<int,int> apply_move(pair<int, int> position, string action);

    bool move_respects_geometry(pair<int, int> position, string action );

    pair<int, int> move_at_random();

    static int compute_distance(pair<int,int> a, pair<int,int> b);

    string choose_action_weighted_probability(vector<string> moves, vector<int> weights);

    string  move_with_preemptive_shield(const delayedShield& shield, VI& init_robot_moves, int delta);

    string move_annoyingly();

    void check_rewards();

    void place_reward_randomly();

    void remove_duplicates(VI& vec);

    void play_naive(VI& init_robot_moves, const delayedShield& shield, int delta);

    void play_safe_or_control_shield(VI& init_robot_moves, const delayedShield& shield, int delta, bool safest);

    void play_robot_desire(VI& init_robot_moves, const delayedShield& shield, int delta);

    string  move_toward_rewards();

    vector<int> compute_distance_towards_reward(const vector<string>& moves);

public:
    int max_delay;
    int score;
    int num_interference;

    chaseGame(int max_delay, bool play_safety_strategy, bool play_controllability_strategy, bool compute_memoryless);

    void print_game(int step);

    void play_game(int num_of_iterations, bool print_each_step, const delayedShield& shield, int delta,
                   bool play_safety_strategy, bool play_controllability_strategy, int print_delay = 80);

    void read_arena();

    delayedShield generate_graph();
};