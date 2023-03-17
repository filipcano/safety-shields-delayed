#include <cstring>
#include <cstdlib>
#include <ctime>
#include <map>
#include "delayedShield.cpp"
#include "chaseGame.cpp"
using namespace std;


int main(int argc,char **argv) {
    bool play_safety_strategy = false;
    bool play_controllability_strategy = false;
    bool compute_memoryless_strategy = false;
    bool play_single_game = true;
    bool dont_play = false;
    int max_delta = 0;
    int steps_of_each_game = 1000;
    int number_of_rounds = 0;
    int initialize_with_file = false;
    int init_state = -1;
    string init_file = " ";
    int seed = 1;
    for (int i = 0 ; i < argc; ++i) {
        string argument = argv[i];
        if (argument == "play_controllable") play_controllability_strategy = true;
        if (argument == "play_safest") play_safety_strategy = true;
        if (argument == "compute_memoryless") compute_memoryless_strategy = true;
        if (argument == "dont_play") dont_play = true;
        string keyword = "delta=";
        if (argument.find(keyword) != std::string::npos) max_delta = stoi(argument.substr(keyword.size(),argument.size()-keyword.size()));
        keyword = "steps_of_each_game=";
        if (argument.find(keyword) != std::string::npos) {
            steps_of_each_game = stoi(argument.substr(keyword.size(),argument.size()-keyword.size()));
        }
        keyword = "number_of_rounds=";
        if (argument.find(keyword) != std::string::npos) {
            number_of_rounds = stoi(argument.substr(keyword.size(),argument.size()-keyword.size()));
        }
        keyword = "seed=";
        if (argument.find(keyword) != std::string::npos) {
            seed = stoi(argument.substr(keyword.size(),argument.size()-keyword.size()));
        }
        keyword = "init_file=";
        if (argument.find(keyword) != std::string::npos) {
            init_file = argument.substr(keyword.size(),argument.size()-keyword.size());
            initialize_with_file = true;
        }
        keyword = "init_state=";
        if (argument.find(keyword) != std::string::npos) {
            init_state = stoi(argument.substr(keyword.size(),argument.size()-keyword.size()));
        }

    }
    if (number_of_rounds > 0) play_single_game = false;

    if (initialize_with_file) {
        if (init_state == -1) {
            cout << "Cannot initialize with file with no init state given\n";
            return 0;
        }

        delayedShield shield = delayedShield();
        cout << init_file << ' ' << init_state << endl;
        shield.initialize_with_file(init_file, init_state, max_delta, play_safety_strategy, play_controllability_strategy, compute_memoryless_strategy);
        shield.run();
        shield.printGraph();
        shield.printStrategy();
        shield.print_safety_vs_controllability_strategies();
        return 0;

    }

    srand (time(NULL));
    srand (seed);
    chaseGame game = chaseGame(max_delta, play_safety_strategy, play_controllability_strategy, compute_memoryless_strategy);
    game.read_arena();
    delayedShield shield = game.generate_graph();
//    shield.print_safety_vs_controllability_strategies();
//    for (int i = 0; i < shield.controllabilityLevel.size(); ++i) cout << shield.controllabilityLevel[i] << ", ";
//    cout << endl;
//    shield.print_for_mathematica();
////    shield.printGraph();
//    return 0;

    if (shield.delta < max_delta) {
        cout << "You asked for max_delay = " << max_delta << " but the game is uncontrollable for delta >= " << shield.delta << endl;
        max_delta = shield.delta;
    }
//    if (dont_play) shield.print_safety_vs_controllability_strategies();
    cout << "controlability levels:\n";
    for (int s = 0; s < shield.controllabilityLevel.size(); ++s) {
        cout << shield.stateNames[s] << " : " << shield.controllabilityLevel[s] << endl;
    }
    if (dont_play) return 0;

    if (play_single_game) {
        game.play_game(1000, true, shield, 0, play_safety_strategy, play_controllability_strategy);
        return 0;
    }
    cout << "\nGame results for trivial strategy,  " << number_of_rounds << " rounds with " << steps_of_each_game << " steps each. \n";
    for (int delta = 0; delta <= max_delta; delta += 2) {
        int score = 0;
        int num_interference = 0;
        for (int i = 0; i < number_of_rounds; ++i) {
            game.play_game(steps_of_each_game, false, shield, delta, false, false);
            score += game.score;
            num_interference += game.num_interference;
        }
        cout << "For delta= " << delta << ", score: " << score << ", num of shield interference: " << num_interference << endl;
    }
    cout << endl;
    if (play_controllability_strategy){
        cout << "game results for most_controllable strategy,  " << number_of_rounds << " rounds with " << steps_of_each_game << " steps each. \n";
        for (int delta = 0; delta <= max_delta; delta += 2) {
            int score = 0;
            int num_interference = 0;
            for (int i = 0; i < number_of_rounds; ++i) {
                game.play_game(steps_of_each_game, false, shield, delta, false, true);
                score += game.score;
                num_interference += game.num_interference;
            }
            cout << "For delta= " << delta << ", score: " << score << ", num of shield interference: " << num_interference << endl;
        }
    }
    cout << endl;
    if (play_safety_strategy){
        cout << "game results for safest strategy,  " << number_of_rounds << " rounds with " << steps_of_each_game << " steps each. \n";
        for (int delta = 0; delta <= max_delta; delta += 2) {
            int score = 0;
            int num_interference = 0;
            for (int i = 0; i < number_of_rounds; ++i) {
                game.play_game(steps_of_each_game, false, shield, delta, true, false);
                score += game.score;
                num_interference += game.num_interference;
            }
            cout << "For delta= " << delta << ", score: " << score << ", num of shield interference: " << num_interference << endl;
        }
    }
}