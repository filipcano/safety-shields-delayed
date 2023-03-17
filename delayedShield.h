//
// Created by fcano on 15.04.21.
//

#pragma once

#include <iostream>
#include <string>
#include <map>
#include <set>
#include <stack>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <utility>
#include <chrono>
#include <unordered_set>
using namespace std;

typedef vector<int> VI;
// A strategy is a set of instructions for each state of the delayed game.
// Since the game is delayed, a state is represented by a pair of (current) state,
// and a list of the delta/2 (+-1) previous actions.
// Strategy[{s, sigmavec}] is a boolean vector, so Strategy[{s, sigmavec}][i] means that move i (as an int) is valid
// in state s and previous moves sigmavec.
typedef map<pair<int,VI>,vector<bool>> Strategy;
typedef map<pair<int,VI>,int> postStrategy;



class delayedShield {

    // Graph[i] represents the node (state) numbered i
    // Graph[i].first is the list of in-nodes of node i, as list of ints.
    // Graph[i].second is the list of out-nodes of node i, as list of ints.
    vector<pair<VI,VI>> Graph;
    // Each edge is represented as a pair<int, int> e, with the notation e.first --> e.second
    // An edge can be represented by many moves, that are listed as list of ints.
    map<pair<int,int>,VI> EdgeLabel;
    // Unsafe states, Agent states and Environment states. For ex. Ustates[i] means that state i is unsafe.
    vector<bool> Ustates, S0states, S1states;
    int s0; // initial state
    // Contains all possible histories of length up to delta of previous states.
    // AllHistoryStates[delta] is a vector, each of its members is a vector of delta/2 (aprox) states
    // This is confused with alpha in the original paper
    vector<vector<VI>> AllHistoryStates;
    VI epsilon = VI(0); // just an empty vector
    map<pair<int,VI>, int> ShrinkStack; // ShrinkStack[{i, sigmavec}] = 1 if Shrink has been applied to it, -1 otherwise
    map<int, int> ShrinkStack_memless; // memoryless version

    /// Main four functions of the class
    void parseGame(string nameOfFile);

    Strategy FPIiteration();

    void Shrink(int s, const VI& sigmavec, Strategy& Xi, Strategy& Xi_prev);

    void Shrink(int s, vector<vector<bool>>& Xi);

    bool delayedStrategies();

    void populate_controllabilityLevel();

    void compute_post_strategies();

    /// Some utility functions

    static pair<string,int> separateArrowSI(string s);

    static pair<int,int> separateArrowII(string s);

    VI states_at_dist_from_state(int init_state, int target_dist, bool forward);

    /// Initialize objects
    void initializeNextAndPrevState();

    void initializeStrategy_rec(Strategy& Xi, VI& sigmavec, const int pos, const int state);

    Strategy initializeStrategy(int local_delta);

    void initAllPha_rec(VI& vec, int pos);

    void initializeAllPha();

    void initializeS0moves();

    /// Printintg utilities

    static void print_vector(const VI& v);

    static void print_vector(const vector<bool>& v);

    VI find_nodes_at_distance_k(int init_state, VI sigmavec, int k, bool forward);

    void clean_strategy(Strategy& Xi);

    vector<vector<bool>> copy_strategy_as_memoryless(const Strategy& Xi, int local_delta);


public:
    int numOfStates, numOfMoves, Delta; // number of states, number of moves and max delay that we will compute
    map<string,int> stateNums; // maps a state as a string with its corresponding int
    vector<string> stateNames; // vector with names of states. statenames[i] gives the name of state numbered i.
    map<string,int> moveNums; // maps a move as a string with its corresponding int
    vector<string> moveNames; // vector with names of moves. moveNames[i] gives the name of move numbered i.
    vector<vector<int>> nextState; // vector with nextState. nextState[s][i] = sprime when the edge s --> sprime is labelled i (all as ints)
    vector<vector<int>> previousState; // vector with previousState. previousState[sprime][i] = s when the edge s --> sprime is labelled i (all as ints)
    vector<Strategy> Strategies; // vector of strategies, Strategies[i] contains strategy for delay i
    vector<vector<vector<bool>>> Strategies_memless; //
    vector<postStrategy> safetyStrategies;
    vector<postStrategy> controllabilityStrategies;
    vector<vector<int>> safetyStrategies_memless;
    vector<vector<int>> controllabilityStrategies_memless;
    bool compute_memoryless;
    // vector of strategies for best fit of agent desire
    // desireStrategies[delta_shield][{ {s_sh, sigmavec}, {delta_agent, s_ag}  }] is int that corresponds to strategy best
    // fitting agent's desire at state s_sh for shield, s_ag for agent and sigmavec as history of moves with delays delta_agent and delta_shield
    vector<map<pair<pair<int, VI>, pair<int,int>>,int>> desireStrategies;
    VI delta_agent_desireStrategies;
    VI delta_shield_desireStrategies;
    vector<vector<VI>> Alpha; // vector of alphas (init moves), Alpha[i] contains a list of vectors of size delta/2 +-1
    int delta;
    // vector of safety level, safetyLevel[s] contains the safetylevel (lenght of shortest path to unsafe region)
    // of a state. Only for safety region in delta = 0.
    vector<int> safetyLevel;
    bool compute_safety_level;
    // vector of safety level, controllabilityLevel[s] contains the controllabilitylevel
    // sum of delta times the number of histories with strategy in delta delay
    // of a state.
    vector<int> controllabilityLevel;
    bool compute_controllability_level;
    vector<bool> S0moves;

    delayedShield();

    void initialize_with_file(string input_file, int init_state, int Delta,
                              bool compute_safety_level, bool compute_controllability_level, bool compute_memoryless);

    void initialize_with_game(int numOfStates, int numOfStrategies, int Delta, int s0,
                              bool compute_safety_level, bool compute_controllability_level, bool compute_memoryless,
                                vector<string>& stateNames, vector<string>& labelNames, map<string,int>& labelNums,
                                vector<pair<VI,VI>>& Graph, map<pair<int,int>,VI>& EdgeLabel, map<string,int>& stateMap,
                                vector<bool>& Ustates, vector<bool>& S0states, vector<bool>& S1states);

    void run();

    pair<int, VI> compute_best_action(const pair<int,VI>& statehist, vector<bool> allowed_moves, int i, string flag);


    void printGraph(bool names = true);

    void printStrategy(const Strategy& Xi, const vector<bool>& vertices, bool print_empty = false);

    void printStrategy(const vector<vector<bool>>& Xi, const vector<bool>& vertices, bool print_empty = false);

    void printStrategy(const Strategy& Xi, bool print_empty = false);

    void printStrategy(const vector<vector<bool>>& Xi, bool print_empty = false);

    void printStrategy(bool memoryless = false, bool print_empty = false);

    void printAlpha();

    void print_safety_vs_controllability_strategies();

    void print_for_mathematica(string FilePath = "");

    bool is_valid_transition(int state0, int state1);

    /// Boolean vector part

    static bool vbool_is_empty(const vector<bool>& v);

    static int vbool_pop(vector<bool>& v);

    static void vbool_insert(vector<bool>& v, const VI& topush);

    static void vbool_insert(vector<bool>& v, int topush);

    static void vbool_delete(vector<bool>& v, int topush);

    static void vbool_delete(vector<bool>& v, const VI& topush);

    static vector<bool> vbool_intersect(const vector<bool>& u, const vector<bool>& v);

    static void remove_duplicates(VI& vec);


};
