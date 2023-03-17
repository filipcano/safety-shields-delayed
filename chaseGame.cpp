#include "chaseGame.h"

chaseGame::chaseGame(int max_delay, bool play_safety_strategy, bool play_controllability_strategy, bool compute_memoryless) {
    basic_moves = {
            {'U', pair<int,int>(0,1)},
            {'D', pair<int,int>(0,-1)},
            {'L', pair<int,int>(-1,0)},
            {'R', pair<int,int>(1,0)},
            {'S', pair<int,int>(0,0)}
    };
    score = 0;
    num_interference = 0;
    this->max_delay = max_delay;
    this->play_controllability_strategy = play_controllability_strategy;
    this->play_safety_strategy = play_safety_strategy;
    this->compute_memoryless = compute_memoryless;
    non_obstacled_locations = vector<pair<int,int>>(0);
}

void chaseGame::print_game(int step) {
    std::system("clear");
    char blockchar = '*';
    char emptychar = ' ';
    for(int i=0;i<M+3;++i) cout << blockchar;
    cout << endl;
    for(int j=N;j >= 0;--j) {
        cout << blockchar;
        for(int i=0;i<= M;++i) {
            pair<int, int> pos = {i,j};
            if (pos_robot.first==i and pos_robot.second==j) cout << 'R';
            else if (pos_kid.first==i and pos_kid.second==j) cout << 'K';
            else if (find(Obstacles.begin(), Obstacles.end(), pos) != Obstacles.end()) cout << blockchar;
            else if (find(Rewards.begin(), Rewards.end(), pos) != Rewards.end()) cout << '$';
            else cout << emptychar;
        }
        cout << blockchar << endl;
    }
    for(int i=0;i<M+3;++i) cout << blockchar;
    cout << endl;
    cout << "Step: " << step << endl;
    cout << "Score: " << score << endl;
    cout << "Num of interference: " << num_interference << endl;
}

string chaseGame::pos2state(pair<int,int> robot, pair<int,int> kid, bool tempkey) {
    string keychar = tempkey ? "r" : "k"; // kid or robot
    string sepchar = "_"; // separator char
    string state = keychar + sepchar + to_string(robot.first) + sepchar + to_string(robot.second) + sepchar + to_string(kid.first) + sepchar + to_string(kid.second);
    return state;
}

void chaseGame::state2pos(string state, pair<int,int>& robot, pair<int,int>& kid, bool& tempkey) {
    assert(state[0] == 'r' or state[0] == 'k');
    string sepchar = "_"; // separator char
    tempkey = state[0] == 'r';
    int prevpos = 1;
    int pos = 0;
    for (int i = 0; i < 4; ++i) {
        pos = state.find(sepchar, prevpos+1);
        if (i == 0)
            robot.first = stoi(state.substr(prevpos+1, pos-prevpos));
        else if (i ==1)
            robot.second = stoi(state.substr(prevpos+1, pos-prevpos));
        else if (i == 2)
            kid.first = stoi(state.substr(prevpos+1, pos-prevpos));
        else if (i == 3)
            kid.second = stoi(state.substr(prevpos+1, pos-prevpos));
        else
            cout << "ERRROR \n\n\n";
        prevpos = pos;
    }
}

pair<int, int> chaseGame::sum(pair<int,int> a, pair<int, int> b) {
    pair<int,int> c;
    c.first = a.first + b.first;
    c.second = a.second + b.second;
    return c;
}

pair<int,int> chaseGame::apply_move(pair<int, int> position, string action) {
    for (char c : action)
        position = sum(position, basic_moves[c]);
    return position;
}

bool chaseGame::move_respects_geometry(pair<int, int> position, string action ) {
    for (int i = 0; i < action.size(); ++i) {
        char c = action[i];
        if (i != action.size()-1)
            key ? Obstacles.push_back(pos_kid) : Obstacles.push_back(pos_robot);
        pair<int,int> newpos = sum(position,basic_moves[c]);
        if (find(Obstacles.begin(), Obstacles.end(), newpos) != Obstacles.end()) {
            if (i != action.size()-1)
                Obstacles.pop_back();
            return false;
        }
        if (newpos.first < 0 or newpos.second < 0 or newpos.first > M or newpos.second > N) {
            if (i != action.size()-1)
                Obstacles.pop_back();
            return false;
        }
        position = newpos;
        if (i != action.size()-1)
            Obstacles.pop_back();
    }
    return true;
}

// deprecated
pair<int, int> chaseGame::move_at_random() {
    vector<string> *current_moves = key ? &robot_moves : &kid_moves;
    pair<int,int> *current_pos = key ? &pos_robot : &pos_kid;

    vector<string> allowed_moves(0);
    for (const string& move : *current_moves) {
        if (move_respects_geometry(*current_pos,move))
            allowed_moves.push_back(move);
    }
    string chosen_move = allowed_moves[rand()%allowed_moves.size()];
    return apply_move(*current_pos, chosen_move);
}

int chaseGame::compute_distance(pair<int,int> a, pair<int,int> b) {
    // computes square of euclidean distance between a and b
    int dist = 0;
    dist += (a.first - b.first)*(a.first - b.first);
    dist += (a.second - b.second)*(a.second - b.second);
    return dist;
}

string chaseGame::choose_action_weighted_probability(vector<string> moves, vector<int> weights) {
    assert(moves.size() == weights.size());
    int alles = 0;
    for (int i = 0; i < weights.size(); ++i) alles += weights[i]+1;
    for (int i = 0; i < weights.size(); ++i)
        weights[i] = alles/max(1,weights[i]);

    vector<int> cumulated_weights(weights.size(), 0);
    cumulated_weights[0] = weights[0];
    for (int i=1; i < weights.size(); ++i)
        cumulated_weights[i] = cumulated_weights[i-1]+weights[i];
    int weightsum = cumulated_weights[weights.size()-1];
    int rint = rand()%weightsum;

    for (int i = 0; i < weights.size(); ++i) {
        if (cumulated_weights[i] > rint) {
            return moves[i];
        }
    }
    return moves[moves.size()-1];
}

vector<int> chaseGame::compute_distance_towards_reward(const vector<string>& moves) {
    VI dists(moves.size(), 0);
    for (int i = 0; i < moves.size(); ++i) {
        pair<int,int> next_pos = apply_move(pos_robot, moves[i]);
        dists[i] = best_dist_towards_reward[next_pos.first][next_pos.second];
        dists[i] = dists[i]*dists[i]*dists[i];
    }
    return dists;
}

string  chaseGame::move_with_preemptive_shield(const delayedShield& shield, VI& init_robot_moves, int delta) {
    assert(key); //only the robot can move with strategy, right now
    int state_as_int = shield.stateNums.at(pos2state(hist_pos_robot[delta], hist_pos_kid[delta], key));
    vector<string> allowed_moves(0);
    vector<string> all_moves(0);
    for (const string &move : robot_moves) {
        if (move_respects_geometry(pos_robot, move))
            all_moves.push_back(move);
    }
    if (delta/2 == hist_robot_moves.size() ) {
        for (const string &move : robot_moves) {
            // Note there is no need to check whether move respects geometry, because it is in the strategy
            if (shield.Strategies[delta].find({state_as_int, hist_robot_moves}) != shield.Strategies[delta].end()) {
                if (shield.Strategies[delta].at({state_as_int, hist_robot_moves})[shield.moveNums.at(move)]) {
                    allowed_moves.push_back(move);
                }
            }
        }
    }
    else {
        /// In the first steps of the game, the move is chosen as an init move (alpha)
        allowed_moves.push_back(shield.moveNames.at(init_robot_moves[0]));
        init_robot_moves.erase(init_robot_moves.begin());
    }

    vector<int> dists = compute_distance_towards_reward(all_moves);
    string best_move = choose_action_weighted_probability(all_moves, dists);
    if (find(allowed_moves.begin(), allowed_moves.end(), best_move) == allowed_moves.end()) {
        vector<int> allowed_dists = compute_distance_towards_reward(allowed_moves);
        best_move = choose_action_weighted_probability(allowed_moves, allowed_dists);
        num_interference++;
    }
    string chosen_move = best_move;
    hist_robot_moves.push_back(shield.moveNums.at(best_move));
    if (hist_robot_moves.size() > delta/2)
        hist_robot_moves.erase(hist_robot_moves.begin());
    debug_robot_moves.push_back(chosen_move);
    return chosen_move;
}

string chaseGame::move_toward_rewards() {
    assert(key); //only the robot can move with strategy, right now
    vector<string> allowed_moves(0);
    for (const string &move : robot_moves) {
        if (move_respects_geometry(pos_robot, move))
            allowed_moves.push_back(move);
    }

    string best_move = allowed_moves[0];
    vector<int> dists(allowed_moves.size(), 0);
    for (int i = 0; i < allowed_moves.size(); ++i) {
        pair<int,int> next_pos = apply_move(pos_robot, allowed_moves[i]);
        dists[i] = best_dist_towards_reward[next_pos.first][next_pos.second];
        dists[i] = dists[i]*dists[i]*dists[i];
//        dists[i] = compute_distance(apply_move(pos_robot, allowed_moves[i]), Rewards[0]);
    }
    best_move = choose_action_weighted_probability(allowed_moves, dists);
    string chosen_move = best_move;
    // This will all be necessary to be done at some point

//    hist_robot_moves.push_back(shield.moveNums.at(best_move));
//    if (hist_robot_moves.size() > delta/2)
//        hist_robot_moves.erase(hist_robot_moves.begin());
//    debug_robot_moves.push_back(chosen_move);
    return chosen_move;
}

string chaseGame::move_annoyingly() {
    // only kid moves annoyingly
    assert(!key);
//    pos_kid = hist_pos_kid[delta];

    vector<string> allowed_moves(0);
    for (const string& move : kid_moves) {
        if (move_respects_geometry(pos_kid,move))
            allowed_moves.push_back(move);
    }
    string best_move = allowed_moves[0];
    vector<int> dists(allowed_moves.size(), 0);
    for (int i = 0; i < allowed_moves.size(); ++i)
        dists[i] = compute_distance(pos_robot, apply_move(pos_kid,allowed_moves[i]));
    best_move = choose_action_weighted_probability(allowed_moves, dists);
    string chosen_move = best_move;
    return chosen_move;
//    return apply_move(pos_kid, chosen_move);
}

void chaseGame::read_arena() {
    int numOfObstacles;
    cin>>M>>N;
    cin >> numOfObstacles;
    Obstacles = vector<pair<int,int>>(numOfObstacles);
    for (int i = 0; i < numOfObstacles; ++i) {
        pair<int,int> p;
        cin >> p.first >> p.second;
        Obstacles[i] = p;
    }
    int numOfKidMoves, numOfRobotMoves;
    cin >> numOfKidMoves;
    kid_moves = vector<string>(numOfKidMoves);
    for (int i = 0; i < numOfKidMoves; ++i)
        cin >> kid_moves[i];
    cin >> numOfRobotMoves;
    robot_moves = vector<string>(numOfRobotMoves);
    for (int i = 0; i < numOfRobotMoves; ++i)
        cin >> robot_moves[i];

    M = M-1;
    N = N-1;

    Rewards = vector<pair<int,int>>(1, {0, 0});
    place_reward_randomly();

    pos_robot = {0,0};
    pos_kid = {M, N};
    key = true;
}

void chaseGame::place_reward_randomly() {
    if (non_obstacled_locations.empty()) {
        for (int i = 0; i <= M; ++i) {
            for (int j = 0; j <= N; ++j) {
                pair<int, int> pos = {i, j};
                if (find(Obstacles.begin(), Obstacles.end(), pos) == Obstacles.end())
                    non_obstacled_locations.push_back(pos);
            }
        }
    }
    int n = non_obstacled_locations.size();
    Rewards[0] = non_obstacled_locations[rand()%n];
    best_move_towards_reward = vector<vector<string>>(M+1, vector<string>(N+1, "*"));
    best_dist_towards_reward = vector<vector<int>>(M+1, vector<int>(N+1, M*N));
    queue<pair<int,int>> Q;
    Q.push(Rewards[0]);
    best_dist_towards_reward[Rewards[0].first][Rewards[0].second] = 0;
    while (!Q.empty()) {
        pair<int, int> current_pos = Q.front();
        int current_dist = best_dist_towards_reward[current_pos.first][current_pos.second];
        Q.pop();
        for (string move : robot_moves) {
            string reverse_move = "";
            for (int i = move.size()-1; i >= 0; --i) {
                char action_to_add = '*';
                if (move[i] == 'U') action_to_add = 'D';
                if (move[i] == 'D') action_to_add = 'U';
                if (move[i] == 'L') action_to_add = 'R';
                if (move[i] == 'R') action_to_add = 'L';
                if (move[i] == 'S') action_to_add = 'S';
                reverse_move.push_back(action_to_add);
            }
            if (move_respects_geometry(current_pos,reverse_move)) {
                pair<int, int> next_pos = apply_move(current_pos,reverse_move);
                if (best_move_towards_reward[next_pos.first][next_pos.second] == "*") {
                    best_move_towards_reward[next_pos.first][next_pos.second] = move;
                    best_dist_towards_reward[next_pos.first][next_pos.second] = current_dist + 1;
                    Q.push(next_pos);
                }
            }
        }
    }
}

void chaseGame::check_rewards() {
    if (!key) return;
    if (pos_robot == Rewards[0]) {
        score++;
        place_reward_randomly();
    }
}

void chaseGame::play_safe_or_control_shield(VI& init_robot_moves, const delayedShield& shield, int delta, bool safest) {
    if (key) {
        int aux;
        int state_as_int;
        string move =  move_toward_rewards();
        if (delta/2 == hist_robot_moves.size() ) {
            state_as_int = shield.stateNums.at(pos2state(hist_pos_robot[delta], hist_pos_kid[delta], key));
            auto it = shield.Strategies[delta].find({state_as_int, hist_robot_moves});
            if ((it != shield.Strategies[delta].end()) and !(it->second)[shield.moveNums.at(move)] ) {
                num_interference++;
                aux = shield.controllabilityStrategies[delta].at({state_as_int, hist_robot_moves});
                if (safest) {
                    move = shield.moveNames[shield.safetyStrategies[delta].at({state_as_int, hist_robot_moves})];
                }
                else {
                    move = shield.moveNames[
                            shield.controllabilityStrategies[delta].at({state_as_int, hist_robot_moves})];
                }
            }
        }
        else {
            /// In the first steps of the game, the move is chosen as an init move (alpha)
            move = shield.moveNames.at(init_robot_moves[0]);
            init_robot_moves.erase(init_robot_moves.begin());
        }

        hist_robot_moves.push_back(shield.moveNums.at(move));
        if (hist_robot_moves.size() > delta/2)
            hist_robot_moves.erase(hist_robot_moves.begin());
        debug_robot_moves.push_back(move);

        pair<int,int> newpos = apply_move(pos_robot, move);
        hist_pos_robot.insert(hist_pos_robot.begin(), newpos);
        if (hist_pos_robot.size() > max_delay+1)
            hist_pos_robot.pop_back();
        pos_robot = newpos;

        hist_pos_kid.insert(hist_pos_kid.begin(), pos_kid);
        if (hist_pos_kid.size() > max_delay+1)
            hist_pos_kid.pop_back();
    }
    else {
        string move = move_annoyingly();
        pair<int,int> newpos = apply_move(pos_kid, move);
        hist_pos_kid.insert(hist_pos_kid.begin(), newpos);
        if (hist_pos_kid.size() > max_delay+1)
            hist_pos_kid.pop_back();
        pos_kid = newpos;

        hist_pos_robot.insert(hist_pos_robot.begin(), pos_robot);
        if (hist_pos_robot.size() > max_delay+1)
            hist_pos_robot.pop_back();
    }

}

/// This corresponds to a preemptive shield for the robot
void chaseGame::play_naive(VI& init_robot_moves, const delayedShield& shield, int delta) {
    if (key) {
        string move = move_with_preemptive_shield(shield, init_robot_moves, delta);
        pair<int,int> newpos = apply_move(pos_robot, move);
        hist_pos_robot.insert(hist_pos_robot.begin(), newpos);
        if (hist_pos_robot.size() > max_delay+1)
            hist_pos_robot.pop_back();
        pos_robot = newpos;

        hist_pos_kid.insert(hist_pos_kid.begin(), pos_kid);
        if (hist_pos_kid.size() > max_delay+1)
            hist_pos_kid.pop_back();
    }
    else {
        string move = move_annoyingly();
        pair<int,int> newpos = apply_move(pos_kid, move);
        hist_pos_kid.insert(hist_pos_kid.begin(), newpos);
        if (hist_pos_kid.size() > max_delay+1)
            hist_pos_kid.pop_back();
        pos_kid = newpos;

        hist_pos_robot.insert(hist_pos_robot.begin(), pos_robot);
        if (hist_pos_robot.size() > max_delay+1)
            hist_pos_robot.pop_back();
    }
}

void chaseGame::play_game(int num_of_iterations, bool print_each_step, const delayedShield& shield, int delta,
                          bool play_safety_strategy, bool play_controllability_strategy, int print_delay) {
    pos_robot = {0,0};
    pos_kid = {M, N};
    hist_pos_robot = vector<pair<int, int>>(max_delay, pos_robot);
    hist_pos_kid = vector<pair<int, int>>(max_delay, pos_kid);
    hist_pos_robot.push_back(pos_robot);
    hist_pos_kid.push_back(pos_kid);
    hist_robot_moves = vector<int>(0);
    debug_history_states = vector<string>(0);
    debug_history_states.push_back(pos2state(pos_robot, pos_kid, key));
    debug_robot_moves = vector<string>(0);
    score = 0;
    num_interference = 0;
    vector<int> init_robot_moves = shield.Alpha[(max_delay+1)/2][0];
    if (print_each_step) print_game(0);
    for (int i=0;i<num_of_iterations;++i) {
        if (play_safety_strategy) {
            play_safe_or_control_shield(init_robot_moves, shield, delta, true);
        }
        else if (play_controllability_strategy) {
            play_safe_or_control_shield(init_robot_moves, shield, delta, false);
        }
        else play_naive(init_robot_moves, shield, delta);

        check_rewards();
        key = not key;
        debug_history_states.push_back(pos2state(pos_robot, pos_kid, key));
        if (print_each_step) {
            cout << endl;
            print_game(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(print_delay));
        }
        if (pos_robot.first == pos_kid.first and pos_robot.second == pos_kid.second) {
            cout << "Crashed! \n";
            break;
        }
    }
//    cout << "Score obtained with delta = " << delta << " is : " << score << endl;
}

void chaseGame::remove_duplicates(VI& vec) {
    sort( vec.begin(), vec.end() );
    vec.erase( unique( vec.begin(), vec.end() ), vec.end() );
}

delayedShield chaseGame::generate_graph() {
    delayedShield shield = delayedShield();

    int numOfStates, numOfStrategies, Delta;
    vector<string> stateNames(0);
    vector<pair<VI,VI>> Graph(0);
    map<pair<int,int>,VI> EdgeLabel;
    vector<string> labelNames;
    map<string,int> labelNums;
    vector<bool> Ustates, S0states, S1states; //dont know yet about initialization
    int s0;

    vector< pair<string,string> > edgeVec(0);
    vector< string > weightVec(0);
    set< string > unsafeSet;
    set< string > stateSet;
    string statecurr;
    string statenext;
    bool tempkey = false;
    queue<string> statesCurr;
    queue<string> statesNext;

    pos_robot = {0,0};
    pos_kid = {M,N};
    key = true;
    string init = pos2state(pos_robot, pos_kid,true);
    statesCurr.push(init);
    stateSet.insert(init);
    cout << M << ' ' << N << endl;
    while(!statesCurr.empty()) {
        while(!statesCurr.empty()) {
            statecurr = statesCurr.front();
            state2pos(statecurr, pos_robot, pos_kid, tempkey);
            vector<string> *current_moves = key ? &robot_moves : &kid_moves;
            pair<int,int> *current_pos = key ? &pos_robot : &pos_kid;
            pair<int,int> *current_pos_other = !key ? &pos_robot : &pos_kid;

            for (string move : *current_moves) {
                if (move_respects_geometry(*current_pos, move)) {
                    pair<int,int> temp_pos = apply_move(*current_pos, move);
                    statenext = key ? pos2state(temp_pos, pos_kid, !tempkey) : pos2state(pos_robot, temp_pos, !tempkey);
                    edgeVec.push_back({statecurr, statenext});
                    weightVec.push_back(move);
                    if (stateSet.find(statenext) == stateSet.end()) {
                        stateSet.insert(statenext);
                        statesNext.push(statenext);
                        if (temp_pos == *current_pos_other) {
                            unsafeSet.insert(statenext);
                        }
                    }
                }
            }
            statesCurr.pop();
        }
        statesCurr = statesNext;
        statesNext = queue<string>();
        key = !key;
    }

    int r = -1;
    int k = 0;
    map<string,int> stateMap;  //renumbering the states
    for (string tempstr : stateSet) {
        int *p = tempstr[0] == 'r' ? &r : &k;
        *p += 2;
        stateMap[tempstr] = *p;
    }

    numOfStates = stateMap.size();
    stateNames = vector<string>(numOfStates+1,"");
    for(auto i : stateMap)
        stateNames[i.second] = i.first;
    Ustates = vector<bool>(stateNames.size(),false);
    S0states = vector<bool>(stateNames.size(),false);
    S1states = vector<bool>(stateNames.size(),false);
    for (int j=1;j<stateNames.size();++j) {
        if (j%2 == 0) S1states[j] = 1;
        else S0states[j] = 1;
    }
    set<string> moveset;
    for (string move : weightVec)
        moveset.insert(move);
    labelNames = vector<string>(0);
    labelNums = map<string,int>();
    for (auto move : moveset) {
        labelNames.push_back(move);
        labelNums[move] = labelNames.size()-1;
    }
    assert(labelNames.size() == labelNums.size());
    numOfStrategies = labelNames.size();
    pair<VI, VI> emptypair(VI(0), VI(0));
    Graph = vector<pair<VI,VI>>(numOfStates+1,emptypair);
    EdgeLabel = map<pair<int,int>,VI>();
    for (int j = 0; j < edgeVec.size(); ++j) {
        pair<int, int> edge(stateMap[edgeVec[j].first], stateMap[edgeVec[j].second]);
        int vertexout = edge.first;
        int vertexin = edge.second;
        Graph[vertexout].second.push_back(vertexin);
        Graph[vertexin].first.push_back(vertexout);

        if (EdgeLabel.find(edge) == EdgeLabel.end()) {
            EdgeLabel[edge] = VI(0);
        }
        EdgeLabel[edge].push_back(labelNums[weightVec[j]]);
    }
    for (auto& vertex : Graph){
        remove_duplicates(vertex.first);
        remove_duplicates(vertex.second);
    }
    for (string ustate : unsafeSet)
        Ustates[stateMap[ustate]] = true;
    Delta = max_delay;
    s0 = stateMap[init];
    shield.initialize_with_game(numOfStates, numOfStrategies, Delta, s0, play_safety_strategy, play_controllability_strategy,
                                compute_memoryless,
                                stateNames, labelNames, labelNums,
                                Graph, EdgeLabel, stateMap, Ustates, S0states, S1states); // faltan delta i s0
    shield.run();
    max_delay = shield.delta;
    return shield;
}
