//
// Created by fcano on 15.04.21.
//

#include "delayedShield.h"

delayedShield::delayedShield() {
    numOfStates = 0;
    numOfMoves = 0;
    s0 = 0;
    Delta = 0;
    delta = 0;
    delta_agent_desireStrategies = VI(0);
    delta_shield_desireStrategies = VI(0);

    compute_safety_level = true;
    compute_controllability_level = true;
//    delta_agent_desireStrategies = VI{4};
//    delta_shield_desireStrategies = VI{2};
}

void delayedShield::initialize_with_file(string input_file, int init_state, int Delta,
                                         bool compute_safety_level, bool compute_controllability_level, bool compute_memoryless) { // deprecated
    s0 = init_state;
    parseGame(input_file);
    this->Delta = Delta;
    this->compute_safety_level = compute_safety_level;
    this->compute_controllability_level = compute_controllability_level;
    this->compute_memoryless = compute_memoryless;
    Strategies = vector<Strategy>(Delta+1);
    Strategies_memless = vector<vector<vector<bool>>>(0);
    Alpha = vector<vector<VI>>((Delta/2)+2, vector<VI>(0) );
    AllHistoryStates = vector<vector<VI>>((Delta / 2) + 1, vector<VI>(0) );
    initializeS0moves();
    initializeAllPha();
    initializeNextAndPrevState();
}

void delayedShield::initialize_with_game(int numOfStates, int numOfStrategies, int Delta, int s0,
                                         bool compute_safety_level, bool compute_controllability_level, bool compute_memoryless,
                                         vector<string>& stateNames, vector<string>& labelNames, map<string,int>& labelNums,
                                         vector<pair<VI,VI>>& Graph, map<pair<int,int>,VI>& EdgeLabel, map<string,int>& stateMap,
                                         vector<bool>& Ustates, vector<bool>& S0states, vector<bool>& S1states) {
    this->numOfStates = numOfStates;
    this->numOfMoves = numOfStrategies;
    this->Delta = Delta;
    this->s0 = s0;
    this->stateNames = stateNames;
    this->Graph = Graph;
    this->EdgeLabel = EdgeLabel;
    this->moveNames = labelNames;
    this->moveNums = labelNums;
    this->Ustates = Ustates;
    this->S0states = S0states;
    this->S1states = S1states;
    this->stateNums = stateMap;
    this->compute_safety_level = compute_safety_level;
    this->compute_controllability_level = compute_controllability_level;
    this->compute_memoryless = compute_memoryless;
    Strategies = vector<Strategy>(Delta+1);
    Alpha = vector<vector<VI>>((Delta/2)+2, vector<VI>(0) );
    AllHistoryStates = vector<vector<VI>>((Delta / 2) + 1, vector<VI>(0) );
    initializeS0moves();
    initializeAllPha();
    initializeNextAndPrevState();
}

void delayedShield::run() {
    for (int i = 0; i < S0states.size(); ++i) {
        assert(!S0states[i] or !S1states[i]);
    }
    Alpha[0].push_back(epsilon);
    Strategies[0] = FPIiteration();
    Strategies_memless.push_back(copy_strategy_as_memoryless(Strategies[0],0));
    delayedStrategies();
    compute_post_strategies();
}

void delayedShield::parseGame(string nameOfFile) {
    string line;
    ifstream nameFileout;
    nameFileout.open(nameOfFile);
    for(int i=0; i<10;++i) {
        getline(nameFileout, line);

        if (i==2) {
            std::vector<std::string> statesVec;
            boost::split(statesVec, line, [](char c){return c == ',';});
            statesVec.pop_back();
            numOfStates = 0;
            for(int j=0;j<statesVec.size();++j){
                numOfStates = max(numOfStates, separateArrowSI(statesVec[j]).second);
            }
            stateNames = vector<string>(numOfStates+1,"");
            for(int j=0;j<statesVec.size();++j){
                auto aux = separateArrowSI(statesVec[j]);
                stateNames[aux.second] = aux.first;
            }
            Ustates = vector<bool>(stateNames.size(),false);
            S0states = vector<bool>(stateNames.size(),false);
            S1states = vector<bool>(stateNames.size(),false);
            for (int j=1;j<stateNames.size();++j) {
                if (j%2 == 0) S1states[j] = 1;
                else S0states[j] = 1;
            }
        }
        if (i==3) {
            int pos = line.find("EdgeWeight",0,10);
            string edgesString = line.substr(7,pos-11);
            string weightsString = line.substr(pos + 15, line.size() - pos - 17);
            std::vector<std::string> edgesVec;
            std::vector<std::string> weightsVec;
            boost::split(edgesVec, edgesString, [](char c){return c == ',';});
            boost::split(weightsVec, weightsString, [](char c){return c == ',';});
            edgesVec.pop_back(); weightsVec.pop_back();
            assert(edgesVec.size() == weightsVec.size());

            set<string> labelset;
            for(int j=0;j<weightsVec.size();++j) {
                weightsVec[j] = weightsVec[j][0] == ' ' ? weightsVec[j].substr(1,string::npos) : weightsVec[j];
                labelset.insert(weightsVec[j]);
            }
            moveNames = vector<string>(0);
            moveNums = map<string,int>();

            for (auto label : labelset) {
                moveNames.push_back(label);
                moveNums[label] = moveNames.size() - 1;
            }
            assert(moveNames.size() == moveNums.size());
            numOfMoves = moveNames.size();

            pair<VI,VI> emptypair(VI(0),VI(0));
            Graph = vector<pair<VI,VI>>(numOfStates+1,emptypair);
            EdgeLabel = map<pair<int,int>,VI>();
            for (int j=0; j < edgesVec.size(); j++) {
                int vertexout = separateArrowII(edgesVec[j]).first;
                int vertexin = separateArrowII(edgesVec[j]).second;
                Graph[vertexout].second.push_back(vertexin);
                Graph[vertexin].first.push_back(vertexout);
                if (EdgeLabel.find(separateArrowII(edgesVec[j])) == EdgeLabel.end()) {
                    EdgeLabel[separateArrowII(edgesVec[j])] = VI{moveNums[weightsVec[j]]};
                }
                else EdgeLabel[separateArrowII(edgesVec[j])].push_back(moveNums[weightsVec[j]]);
            }
            for (auto& vertex : Graph){
                remove_duplicates(vertex.first);
                remove_duplicates(vertex.second);
            }


        }
        if (i==4) {
            int pos = 15;
            line = line.substr(pos,string::npos);
            vector<string> uvec;
            boost::split(uvec, line, [](char c){return c == ',';});
            uvec.pop_back();
            for (int j=0;j<uvec.size();++j) Ustates[stoi(uvec[j])] = 1;
        }
    }
}

Strategy delayedShield::FPIiteration() {
    auto clock=std::chrono::high_resolution_clock();
    Strategy Xi;
    vector<bool> currentU;
    currentU = Ustates;
    auto begin =clock.now();
    for(int s=0;s<S0states.size();++s) {
        if (S0states[s]) {
            if (Ustates[s]) {
                //do nothing
                Xi[{s,epsilon}] = vector<bool>(numOfMoves, false);
            }
            else {
                vector<bool> validStrategies(numOfMoves, false);
                VI s_outedges = Graph[s].second;
                for (int sprime : s_outedges) {
                    for (int act : EdgeLabel[{s, sprime}])
                        validStrategies[act] = true;
                }
                Xi[{s,epsilon}] = validStrategies;
            }
        }
    }

    vector<bool> historicalU = currentU;
    stack<int> currentU_stack;
    for (int s = 0; s < currentU.size(); ++s) {
        if (currentU[s])
            currentU_stack.push(s);
    }
    while(!currentU_stack.empty()) {
//        int sprime = vbool_pop(currentU);
        int sprime = currentU_stack.top();
        currentU_stack.pop();
        historicalU[sprime] = true;
        VI topush(0);
        if (S0states[sprime]) {
            VI inedges = Graph[sprime].first;
            for (int s : inedges) {
                if (!historicalU[s]) {
//                    topush.push_back(s);
                    currentU_stack.push(s);
                    historicalU[s] = true;
                }
            }
//            vbool_insert(currentU, topush);
        }
        else if (S1states[sprime]){
            VI inedges = Graph[sprime].first;
            for(int s : inedges) {
                for (int sigma : EdgeLabel[{s,sprime}]) {
                    if (Xi.find({s, epsilon}) == Xi.end())
                        Xi[{s, epsilon}] = vector<bool>(numOfMoves, false);
                    vbool_delete(Xi[{s, epsilon}], sigma);
                    if (vbool_is_empty(Xi[{s, epsilon}]) and !historicalU[s]) {
                        topush.push_back(s);
                        currentU_stack.push(s);
                        historicalU[s] = true;
                    }
                }
            }
//            vbool_insert(currentU,topush);
        }
    }
    double duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock.now()-begin).count()/1000.0;
    cout << "Finished with strategy delta = " << delta << " in " << duration << " seconds  \n";
    Ustates = historicalU;
    bool allsome = true;
    for (auto xx : Xi) {
        allsome = allsome and !xx.second.empty();
    }
    assert(allsome);

    begin = clock.now();
    //populate safetyLevel vector
    if (compute_safety_level) {
        safetyLevel = vector<int>(Ustates.size(), -1); //
        for (int i = 0; i < safetyLevel.size(); ++i) {
            if (Ustates[i])
                safetyLevel[i] = 0;
        }

        bool states_left_to_visit = true;
        int current_safety_level = 0;
        while (states_left_to_visit) {
            states_left_to_visit = false;
            for (int i = 0; i < safetyLevel.size(); ++i) {
                int aux3 = safetyLevel[i];
                if (safetyLevel[i] == current_safety_level) {
                    for (int s : Graph[i].first) {
                        string aux1 = stateNames[s];
                        string aux2 = stateNames[i];
                        if (stateNames[s] == "r_4_1_2_3") {
                            cout << "";
                        }
                        if (safetyLevel[s] == -1) {
                            states_left_to_visit = true;
                            safetyLevel[s] = current_safety_level + 1;
                        }
                    }
                }
            }
            current_safety_level++;
        }
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock.now()-begin).count()/1000.0;
        cout << "Finished populating safety vector in " << duration << " seconds  \n";
    }
    return Xi;

}


//the memoryless method
void delayedShield::Shrink(int s, vector<vector<bool>>& Xi) {
    int backward_dist = delta%2 == 0 ? delta -1 : delta;
    if ((S0states[s] and delta%2 == 0) or (S1states[s] and delta%2 == 1)) {
        for (int sprime : Graph[s].first) {
            for (int stilde : Graph[sprime].first) {
                VI sigmalabels = delta%2 == 0 ? EdgeLabel[{sprime, s}] : EdgeLabel[{stilde, sprime}];
                VI prev_states = states_at_dist_from_state(stilde, backward_dist, false);
                for (int sbar : prev_states) {
                    if (!vbool_is_empty(Xi[sbar])) {
                        for (int sigma : sigmalabels) {
                            vbool_delete(Xi[sbar], sigma);
                        }
                        if (vbool_is_empty(Xi[sbar])) {
                            ShrinkStack_memless[sbar] = 1;
                        }
                    }
                }
            }
        }
    }
    else assert(true);
}

void delayedShield::Shrink(int s, const VI& sigmavec, Strategy& Xi, Strategy& Xi_prev) {
    int n = sigmavec.size();
    int sigma_n = -1;
    if (n > 0)
        sigma_n = sigmavec[n-1];  //yes, sigma_n is sigmavec[n-1] because it starts from zero
    if (stateNames[s] == "c_102_4_8") {
        cout << "";
    }

    for (int sprime : Graph[s].first) {

        for (int sigma : EdgeLabel[{sprime,s}]) {
            if (n == 0)
                sigma_n = sigma;
            VI newsigmavec = sigmavec;
            newsigmavec.insert(newsigmavec.begin(), sigma);
            newsigmavec.pop_back();

            if (!Xi_prev[{sprime,newsigmavec}].empty() and Xi_prev[{sprime,newsigmavec}][sigma_n]) {
                vbool_delete(Xi_prev[{sprime,newsigmavec}], sigma_n);
                for (int stilde : Graph[sprime].first) {
                    if (!Ustates[stilde]){
                        if (Xi.find({stilde,newsigmavec}) != Xi.end()) {
                            vbool_delete(Xi[{stilde, newsigmavec}], sigma_n);
                            if (vbool_is_empty(Xi[{stilde, newsigmavec}])) {
                                if (ShrinkStack.find({stilde, newsigmavec}) == ShrinkStack.end())
                                    ShrinkStack[{stilde, newsigmavec}] = 1;
                            }
                        }
                        else {
                            if (ShrinkStack.find({stilde, newsigmavec}) == ShrinkStack.end())
                                ShrinkStack[{stilde, newsigmavec}] = 1;
                        }
                    }
                }
            }
        }
    }
}


bool delayedShield::delayedStrategies() {
    delta = 0;
    auto clock=std::chrono::high_resolution_clock();

    while (delta < Delta) {

        auto begin=clock.now();
        ShrinkStack = map<pair<int,VI>, int>();
        Strategy Xi = initializeStrategy(delta+1); //the new strategy
        Strategy Xi_prev = Strategies[delta];

        if (delta%2 == 0) {
            vector<VI> alpha(0); // the new alpha
            for (int s=0; s< S0states.size();++s) {
                if (S1states[s]) {
                    for (const VI &sigmavec : AllHistoryStates[delta / 2]) { // this may be delta/2 +1
                        if (!Ustates[s]) {
                            vector<bool> newactions(numOfMoves, true);
                            bool fresh_unwinnable = true;
                            for (int sprime : Graph[s].second) {
                                if (Xi_prev.find({sprime, sigmavec}) == Xi_prev.end()) {

                                    Xi_prev[{sprime, sigmavec}] = vector<bool>(numOfMoves, false);
                                }
                                auto auxu = Xi_prev[{sprime, sigmavec}];
                                fresh_unwinnable = fresh_unwinnable and !vbool_is_empty(Xi_prev[{sprime, sigmavec}]);
                                vector<bool> aux = Xi_prev[{sprime, sigmavec}];
                                newactions = vbool_intersect(newactions, aux);
                            }
                            Xi.erase({s, sigmavec});
                            Xi[{s, sigmavec}] = newactions;
                            fresh_unwinnable = fresh_unwinnable and vbool_is_empty(newactions);
                            if (fresh_unwinnable and delta >= 0) {
                                if (ShrinkStack.find({s, sigmavec}) == ShrinkStack.end())
                                    Shrink(s, sigmavec, Xi, Xi_prev);
                            }
                        } else {
                            Xi.erase({s, sigmavec});
                            Xi[{s, sigmavec}] = vector<bool>(numOfMoves, false);
                        }
                    }
                }

            }
            bool nothing_to_shrink = false;
            while (not nothing_to_shrink) {
                nothing_to_shrink = true;
                for (auto it = ShrinkStack.begin(); it != ShrinkStack.end(); ++it) {
                    if (it->second == 1) {
                        nothing_to_shrink = false;
                        it->second = -1;
                        Shrink((it->first).first, (it->first).second, Xi, Xi_prev);
                    }
                }
            }

            for (const VI& sigmavec : AllHistoryStates[delta/2]) {
                for (int sprime : Graph[s0].second) {
                    if(!vbool_is_empty(Xi[{sprime, sigmavec}])) {
                        for (int sigma0 : EdgeLabel[{s0,sprime}]) {
                            VI newalpha = sigmavec;
                            newalpha.insert(newalpha.begin(), sigma0);
                            alpha.push_back(newalpha);
                        }
                    }
                }
            }
            Alpha[(delta/2)+1] = alpha;
            if (alpha.empty()) {
                auto end=clock.now();
                double duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count()/1000.0;
                cout << "Uncontrollable with delay " << delta+1 << " in " << duration << " seconds  \n";
                return false;
            }
        }
        else {
            for (int s=0; s<S0states.size(); ++s) {
                if (S0states[s]) {
                    for (const VI &sigmavec : AllHistoryStates[(delta - 1) / 2]) {
                        if (!Ustates[s]) {
                            for (int sprime : Graph[s].second) {
                                for (int sigma0 : EdgeLabel[{s, sprime}]) {
                                    VI newsigma = sigmavec;
                                    newsigma.insert(newsigma.begin(), sigma0);
                                    Xi[{s, newsigma}] = Xi_prev[{sprime, sigmavec}];
                                }
                            }
                        } else {
                            // I would do nothing here
                            for (int sigma0 = 0; sigma0 < numOfMoves; ++sigma0) {
                                VI newsigma = sigmavec;
                                newsigma.insert(newsigma.begin(), sigma0);
                                Xi[{s, newsigma}] = vector<bool>(numOfMoves, false);
                            }
                        }
                    }
                }
            }
        }



//        bool allsome = true;
//        for (auto xx : Xi) {
//            allsome = allsome and !xx.second.empty();
//            if (xx.second.empty()) {
//                cout << xx.first.first << ", {";
//                print_vector(xx.first.second);
//                cout << "}\n";
//            }
//        }
//        assert(allsome);

        clean_strategy(Xi);
//        Strategies[++delta] = Xi;
        Strategies[++delta] = std::move<>(Xi);
        if (compute_memoryless)
            Strategies_memless.push_back(copy_strategy_as_memoryless(Strategies[delta], delta));
        auto end=clock.now();
        double duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count()/1000.0;

        cout << "Finished with strategy delta = " << delta << " in " << duration << " seconds  \n";
    }
    return true;
}

VI delayedShield::find_nodes_at_distance_k(int init_state, VI sigmavec, int k, bool forward) {
    unordered_set<int> nodes1;
    unordered_set<int> nodes2;

    nodes1.insert(init_state);
    bool agent_state = S0states[init_state];
    if (not forward) agent_state = !agent_state;
    for (int i = 0; i < k; ++i) {
        if (agent_state) {
            for (int s : nodes1) {
                if (forward)
                    nodes2.insert(nextState[s][sigmavec[0]]); //this correponded to half of the time. Now better
                else
                    nodes2.insert(previousState[s][sigmavec[0]]); //this correponded to half of the time. Now better
            }
            sigmavec.erase(sigmavec.begin());
        }
        else {
            for (int s : nodes1) {
                if (forward) {
                    for (int sprime : Graph[s].second)
                        nodes2.insert(sprime);
                }
                else {
                    for (int sprime : Graph[s].first)
                        nodes2.insert(sprime);
                }
            }
        }
        nodes1 = nodes2;
        nodes2.clear();
        agent_state = !agent_state;
    }
    VI result(0);
    for (int s : nodes1)
        result.push_back(s);
    return result;
}

//the memoryless version
VI delayedShield::states_at_dist_from_state(int init_state, int target_dist, bool forward) {
    unordered_set<int> nodes1;
    unordered_set<int> nodes2;
    nodes1.insert(init_state);
    for (int current_dist = 0; current_dist < target_dist; ++ current_dist) {
        for (int start_node : nodes1) {
            if (forward) {
                for (int next_node : Graph[start_node].second) {
                    nodes2.insert(next_node);
                }
            }
            else {
                for (int next_node : Graph[start_node].first) {
                    nodes2.insert(next_node);
                }
            }
        }
        nodes1.swap(nodes2);
        nodes2.clear();
    }
    VI result(0);
    for (int s : nodes1) result.push_back(s);
    return result;
}

pair<int, VI> delayedShield::compute_best_action(const pair<int,VI>& statehist, vector<bool> allowed_moves, int i, string flag) {
    bool print = false;
//    if (stateNames.at(statehist.first) == "r_0_0_0_2" and i == 4) {
//        print = true;
//    }
    if (print) {
        cout << "In computing best action wiht delay "<< i << " for state " << stateNames[statehist.first] << "; ";
        for (int j = 0; j < statehist.second.size(); ++j) cout << moveNames[statehist.second[j]] << ", ";
        cout << "], flag " << flag;
    }
    assert(flag == "safety" or flag == "controllability");
    int best_action = -1;
    int max_score = 0;
    VI scores = VI(allowed_moves.size(), 0);
    for (int act = 0; act < allowed_moves.size(); ++act) {
        if (allowed_moves[act]) {
            if (print) cout << "\nFor action " << moveNames[act] << ":\n";
            VI sigmavec = statehist.second;
            sigmavec.push_back(act);
            VI nodes = find_nodes_at_distance_k(statehist.first, sigmavec, i+2, true);
            int aux_score = 0;
            for (int s : nodes) {
                if (print) {
                    cout << stateNames[s] << ": ";
                    if (flag == "safety") cout << safetyLevel[s];
                    else cout << controllabilityLevel[s];
                    cout << endl;
                }
                if (flag == "safety") aux_score += safetyLevel[s];
                else aux_score += controllabilityLevel[s];
            }
            scores[act] = aux_score;
            if (aux_score > max_score) {
                max_score = aux_score;
                best_action = act;
            }
        }
    }
    return {best_action, scores};
}


void delayedShield::compute_post_strategies() {
    auto clock=std::chrono::high_resolution_clock();

    // TODO: Both safety and controllability have very similar structure, maybe we could unify them
    // TODO: Also, find_nodes_at_distance_k seems to be super inefficient, but it's probably fine.
    if (compute_safety_level) {
        safetyStrategies = vector<postStrategy>(delta+1);
        safetyStrategies_memless = vector<vector<int>>(delta+1, VI(numOfStates, -1));
        for (int i = 0; i < safetyStrategies.size(); ++i){
            auto begin=clock.now();
            for (auto it = Strategies[i].begin(); it != Strategies[i].end(); ++it) {
                if (!vbool_is_empty(it->second)) {
                    int best_action = compute_best_action(it->first, it->second, i, "safety").first;
                    safetyStrategies[i][it->first] = best_action;
                }
            }
            // the memoryless part
            if (compute_memoryless) {
                for (int state = 0; state < Strategies_memless[i].size(); ++state) {
                    vector<bool> current_strategy = Strategies_memless[i][state];
                    if (!vbool_is_empty(current_strategy)) {
                        int best_action = -1;
                        int max_safety = 0;
                        for (int act = 0; act < current_strategy.size(); ++act) {
                            if (current_strategy[act]) {
                                VI nodes = states_at_dist_from_state(state, i + 2, true);
                                int aux_safety = 0;
                                for (int s : nodes)
                                    aux_safety += safetyLevel[s];
                                if (aux_safety > max_safety) {
                                    max_safety = aux_safety;
                                    best_action = act;
                                }
                            }
                        }
                        safetyStrategies_memless[i][state] = best_action;
                    }
                }
            }
            auto end=clock.now();
            double duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count()/1000.0;
            cout << "Finished with safety strategy delta = " << i << " in " << duration << " seconds  and size " << safetyStrategies[i].size() << "  \n";
        }
    }
    if (compute_controllability_level) {
        populate_controllabilityLevel();
        controllabilityStrategies = vector<postStrategy>(delta+1);
        controllabilityStrategies_memless = vector<vector<int>>(delta+1, VI(numOfStates, -1));
        for (int i = 0; i < controllabilityStrategies.size(); ++i){
            int max_diff = 0;
            int better_state = 0;
            VI better_vec;
            auto begin=clock.now();
            for (auto it = Strategies[i].begin(); it != Strategies[i].end(); ++it) {
                if (!vbool_is_empty(it->second)) {
                    auto aux = compute_best_action(it->first, it->second, i, "controllability");
                    int best_action = aux.first;
                    auto v = aux.second;
                    std::sort(v.begin(), v.end());
                    int uniqueCount = std::unique(v.begin(), v.end()) - v.begin();
                    if (uniqueCount > max_diff) {
                        max_diff = uniqueCount;
                        better_state = (it->first).first;
                        better_vec = v;
                    }
//                    int best_action = compute_best_action(it->first, it->second, i, "controllability").first;
                    controllabilityStrategies[i][it->first] = best_action;
                }
            }
            if (i == 2) {
                cout << "\n State " << stateNames[better_state] << " with " << max_diff << " different moves.\n ";
                for (auto v : better_vec) cout << v << ' ';
                cout << endl;
            }
            if (compute_memoryless) {
                for (int state = 0; state < Strategies_memless[i].size(); ++state) {
                    vector<bool> current_strategy = Strategies_memless[i][state];
                    if (!vbool_is_empty(current_strategy)) {
                        int best_action = -1;
                        int max_controllability = 0;
                        for (int act = 0; act < current_strategy.size(); ++act) {
                            if (current_strategy[act]) {
                                VI nodes = states_at_dist_from_state(state, i + 2, true);
                                int aux_controllability = 0;
                                for (int s : nodes)
                                    aux_controllability += controllabilityLevel[s];
                                if (aux_controllability > max_controllability) {
                                    max_controllability = aux_controllability;
                                    best_action = act;
                                }
                            }
                        }
                        controllabilityStrategies_memless[i][state] = best_action;
                    }
                }
            }
            auto end=clock.now();
            double duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count()/1000.0;
            cout << "Finished with controllability strategy delta = " << i << " in " << duration << " seconds and size " << controllabilityStrategies[i].size() << "\n";
        }
    }
    desireStrategies = vector<map<pair<pair<int, VI>, pair<int,int>>,int>>(Strategies.size());
    for (int delta_shield : delta_shield_desireStrategies) {
        if (delta_shield <= delta) {
            desireStrategies[delta_shield] = map<pair<pair<int, VI>, pair<int,int>>,int>();
            for (int delta_agent : delta_agent_desireStrategies) {
                for (auto it = Strategies[delta_agent].begin(); it != Strategies[delta_shield].end(); ++it) {
                    int s_sh = it->first.first;
                    VI sigmavec_sh = it->first.second;
                    VI sigmavec = sigmavec_sh;
                    int strategies_to_add = (delta_agent - delta_shield)/2;
                    for ( VI sigmavec_diff : AllHistoryStates[strategies_to_add]) {
                        VI states_ag = find_nodes_at_distance_k(s_sh, sigmavec_diff, 2*strategies_to_add, false);
                    }
                }
            }
        }
    }
}

void delayedShield::populate_controllabilityLevel() {
    auto clock=std::chrono::high_resolution_clock();
    auto begin =clock.now();

    controllabilityLevel = VI(Ustates.size(), -1);
    bool pure_controllabilty = true;
    if (pure_controllabilty) {
        for (int tempdelta = 0; tempdelta <= delta; ++tempdelta) {
            for (auto it = Strategies[tempdelta].begin(); it != Strategies[tempdelta].end(); ++it) {
                int aux = it->first.first;
                if (!vbool_is_empty(it->second))
                    controllabilityLevel[(it->first).first] = tempdelta+1; // plus one, if not some of them may have controllability of -1
            }
        }
    }
    else {
        for (int tempdelta = 0; tempdelta <= delta; ++tempdelta) {
            for (auto it = Strategies[tempdelta].begin(); it != Strategies[tempdelta].end(); ++it) {
                int aux = it->first.first;
                if (!vbool_is_empty(it->second))
                    controllabilityLevel[(it->first).first] += tempdelta+1; // plus one, if not some of them may have controllability of -1
            }
        }
    }
    auto end=clock.now();
    double duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count()/1000.0;
    cout << "Finished populate controllability vector in " << duration << " seconds\n";
}



/// Some utility functions

pair<string,int> delayedShield::separateArrowSI(string s) {
    pair<string,int> result;
    int lenghtOfArrow = 2;
    int pos = s.find("->",0,lenghtOfArrow);
    result.first = s[0] == ' ' ? s.substr(1,pos-1) : s.substr(0,pos);
    result.second = stoi(s.substr(pos+lenghtOfArrow,s.size()-pos+lenghtOfArrow));
    return result;
}

pair<int,int> delayedShield::separateArrowII(string s) {
    pair<int,int> result;
    int lenghtOfArrow = 4;
    int pos = s.find(" -> ",0,lenghtOfArrow);
    //cout << s.substr(0,pos) << ' ' << s.substr(pos+lenghtOfArrow,s.size()-pos+lenghtOfArrow);
    result.first = stoi(s.substr(0,pos));
    result.second = stoi(s.substr(pos+lenghtOfArrow,s.size()-pos+lenghtOfArrow));
    return result;
}

void delayedShield::remove_duplicates(VI& vec) {
    sort( vec.begin(), vec.end() );
    vec.erase( unique( vec.begin(), vec.end() ), vec.end() );
}

vector<vector<bool>> delayedShield::copy_strategy_as_memoryless(const Strategy& Xi, int local_delta) {
    vector<vector<bool>> Xiless(stateNames.size(), vector<bool>(numOfMoves, false));
    vector<bool> touched_states(stateNames.size(), false);
    ShrinkStack_memless.clear();
    assert(Xiless.size() == stateNames.size());
    for (auto it = Xi.begin(); it != Xi.end(); ++it) {
        bool legal_strategy = true;
        for (int sigma : it->first.second) legal_strategy = legal_strategy and S0moves[sigma];
        int state = (it->first).first;
        bool right_player = (S0states[state] and local_delta%2 == 0) or (S1states[state] and local_delta%2 == 1);
        legal_strategy = legal_strategy and right_player;
        if (legal_strategy) {
            if (!touched_states[state] and !vbool_is_empty(it->second)) {
                touched_states[state] = true;
                Xiless[state] = vector<bool>(numOfMoves, true);
            }
            if (!vbool_is_empty(it->second)) {
                Xiless[state] = vbool_intersect(Xiless[state], it->second);
                if (vbool_is_empty(Xiless[state])) {
                    ShrinkStack_memless[state] = 1;
                }
            }
        }
    }

    bool nothing_to_shrink = false;
    while (not nothing_to_shrink) {
        nothing_to_shrink = true;
        for (auto it = ShrinkStack_memless.begin(); it != ShrinkStack_memless.end(); ++it) {
            if (it->second == 1) {
                nothing_to_shrink = false;
                it->second = -1;
                Shrink((it->first), Xiless);
            }
        }
    }

//    int count = 0;
//    for (int i = 0; i < Xiless.size(); ++i) {
//        if (!vbool_is_empty(Xiless[i])) ++count;
//    }
//    cout << "The memoryless strategy contains " << count << " strats. Shrink stack contains " << ShrinkStack_memless.size() << " \n";
    return Xiless;
}

void delayedShield::clean_strategy(Strategy& Xi) {
    stack<pair<int,vector<int>>> states_to_clean;
    for (auto it = Xi.begin(); it != Xi.end(); ++it) {
        if (it->second.empty() or vbool_is_empty(it->second))
            states_to_clean.push(it->first);
    }
    while (!states_to_clean.empty()) {
        auto a = states_to_clean.top();
        states_to_clean.pop();
        Xi.erase(a);
    }
}

/// Printing utilities

void delayedShield::printAlpha()  {
    for (int i = 0; i < Alpha.size(); ++i) {
        cout << "For delay delta = ";
        i == 0 ? cout << i : cout << 2*i-1 << " and " << 2*i;
        cout << " :\n";
        for (const auto& vec : Alpha.at(i)) {
            for (int s : vec) {
                cout << s << "<->" << moveNames[s] << ", ";
            }
            cout << endl;
        }
        cout << "****************\n";
    }
}

void delayedShield::print_vector(const VI& v) {
    for (int i=0; i < v.size(); ++i) cout << v[i] << ", ";
    cout << endl;
}

void delayedShield::print_vector(const vector<bool>& v) {
    for (int i=0; i < v.size(); ++i) cout << v[i] << ", ";
    cout << endl;
}

void delayedShield::printGraph(bool names) {
    cout << "State names" << endl;
    for (int i=0;i < stateNames.size(); ++i) {
        cout << i << " : " << stateNames[i] << "  ";
        if (Ustates[i]) cout << "(Unsafe)";
        cout << endl;
    }
    cout << "\nEdge labels: \n";
    for (auto it = EdgeLabel.cbegin(); it != EdgeLabel.cend();++it) {
        if (names)
            cout << stateNames[it->first.first] << " -> " << stateNames[it->first.second] << " : {";
        else
            std::cout << it->first.first << " -> " << it->first.second << " : {";
        if (names) {
            for (int j : it->second) cout << moveNames[j] << ", ";
        }
        else {
            for (int j : it->second) cout << j << ", ";
        }

        cout << "}\n";
    }
    cout << "\nGraph : \n";
    for (int i=0; i<Graph.size(); ++i) {
        auto vertex = Graph[i];
        VI inlist = vertex.first;
        VI outlist = vertex.second;
        if (names) cout << stateNames[i] << " : ";
        else cout << i << " : ";
        if (names) {
            for (int j : inlist) cout << stateNames[j] << ", ";
            cout << endl;
        }
        else
            print_vector(inlist);
        cout << " || ";
        if (names) {
            for (int j : outlist) cout << stateNames[j] << ", ";
            cout << endl;
        }
        else
            print_vector(outlist);
        cout << endl;
    }
}

void delayedShield::printStrategy(const Strategy& Xi, const vector<bool>& vertices, bool print_empty) {
    int j = 0;
    for (auto statestrat : Xi) {
        if (!vbool_is_empty(statestrat.second) or print_empty) {
            if (vertices[statestrat.first.first]) {
//                if (++j > vertices.size()) return;
                cout << "state : " << statestrat.first.first << " <--> " << stateNames[statestrat.first.first] << "; ";
                for (int id = 0; id < statestrat.first.second.size(); ++id) cout << moveNames[statestrat.first.second[id]] << ',';
                cout << " -> {";
                for (int i = 0; i < statestrat.second.size(); ++i) {
                    if (statestrat.second[i]) cout << moveNames[i] << ',';
                }
                cout << "}\n";
            }
        }
    }
}

void delayedShield::printStrategy(const vector<vector<bool>>& Xi, const vector<bool>& vertices, bool print_empty) {
    for (int i = 0; i < Xi.size(); ++i) {
        if (!vbool_is_empty(Xi[i]) or print_empty) {
            if (vertices[i]) {
                cout << "state : " << i << " <--> " << stateNames[i] << ";  -> {";
                for (int j = 0; j < Xi[i].size(); ++j) {
                    if (Xi[i][j]) cout << moveNames[j] << ',';
                }
                cout << "}\n";
            }
        }
    }
}

void delayedShield::printStrategy(const Strategy& Xi, bool print_empty) {
    vector<bool> vertices(S0states.size(),true);
    printStrategy(Xi, vertices, print_empty);
}

void delayedShield::printStrategy(const vector<vector<bool>>& Xi, bool print_empty) {
    vector<bool> vertices(S0states.size(),true);
    printStrategy(Xi, vertices, print_empty);
}

void delayedShield::printStrategy(bool memoryless, bool print_empty) {
    if (memoryless) {
        for (int i=0; i<Strategies_memless.size(); ++i) {
            cout << "*****************************\n";
            cout << "*** For delay delta = " << i << " : ***\n";
            cout << "*****************************\n";
            printStrategy(Strategies_memless[i], print_empty);
            cout << endl;
        }
    }
    else {
        for (int i=0; i<Strategies.size(); ++i) {
            cout << "*****************************\n";
            cout << "*** For delay delta = " << i << " : ***\n";
            cout << "*****************************\n";
            printStrategy(Strategies[i], print_empty);
            cout << endl;
        }
    }
    cout << endl;
}

void delayedShield::print_safety_vs_controllability_strategies() {
    if (!compute_safety_level or !compute_controllability_level) return;
    for (int i = 0; i <= delta; ++i) {
        cout << "\n\nFor i = " << i << " \n";
        for (auto it = safetyStrategies[i].begin(); it != safetyStrategies[i].end(); ++it) {
            auto it2 = controllabilityStrategies[i].find(it->first);
            if (it2 != controllabilityStrategies[i].end()) {
                int itsec = it->second;
                int it2sec = it2->second;
                if (it->second != it2->second) {
                    VI safety_levels = compute_best_action(it->first, Strategies[i].at(it->first), i, "safety").second;
                    VI controllability_levels = compute_best_action(it->first, Strategies[i].at(it->first), i, "controllability").second;
                    if ((safety_levels[itsec] > safety_levels[it2sec]) and (controllability_levels[itsec] < controllability_levels[it2sec])) {
                        cout << stateNames[it->first.first] << "; [";
                        for (int j: it->first.second) cout << moveNames[j] << ", ";
                        cout << "] : " << moveNames[it->second] << " : " << moveNames[it2->second] << endl;
                        cout << "Safety levels: \n";
                        for (int act = 0; act < safety_levels.size(); ++act) cout << moveNames[act] << ": " << safety_levels[act] << ", ";
                        cout << "\nControllability levels: \n";
                        for (int act = 0; act < controllability_levels.size(); ++act) cout << moveNames[act] << ": " << controllability_levels[act] << ", ";
                        cout << endl << endl;
                    }
                }
            }
        }
    }
}

void delayedShield::print_for_mathematica(string FilePath) {

    cout << "Graph for mathematica code: \n Graph[{";
    vector<string> EdgeWeight(0);
    bool firstime = true;
    for (auto it = EdgeLabel.begin(); it != EdgeLabel.end(); ++it) {
        for (int action : it->second) {
            if (firstime) firstime = false;
            else cout << ", ";
            cout << it->first.first << " -> " << it->first.second;
            if (S0states[it->first.first])
                EdgeWeight.push_back(moveNames[action]);
            else
                EdgeWeight.push_back("u");
        }
    }
    firstime = true;
    cout << "},\nEdgeWeight -> {";
    for (int i = 0; i < EdgeWeight.size(); ++i) {
        if (firstime) firstime = false;
        else cout << ", ";
        cout << EdgeWeight[i];
    }
    firstime = true;
    cout << "}];\ninit = {"  << s0 << "};\n";
    cout << "bad = {";
    for (int i = 0; i < Ustates.size(); ++i) {
        if (Ustates[i]) {
            if (firstime) firstime = false;
            else cout << ", ";
            cout << i;
        }
    }
    cout << ";\n";
}

bool delayedShield::is_valid_transition(int state0, int state1) {
    cout << "\nTransition check for states " << stateNames[state0] << " and " << stateNames[state1] << endl;
    cout << " -- state0 outstates: ";
    for (int outstate : Graph[state0].second) {
        cout << stateNames[outstate] << ", ";
    }
    cout << "\n -- state1 instates: ";
    for (int instate : Graph[state1].first) {
        cout << stateNames[instate] << ", ";
    }
    cout << endl;
    if ((S0states[state0] and S1states[state1]) or (S0states[state1] and S1states[state0])) {
        for (int outstate : Graph[state0].second) {
            if (outstate == state1) return true;
        }
        return false;
    }
    for (int middleoutstate : Graph[state0].second) {
        for (int outstate : Graph[middleoutstate].second) {
            if (outstate == state1) return true;
        }
    }
    return false;
}

/// Initialize objects

void delayedShield::initializeNextAndPrevState() {
    nextState = vector<vector<int>>(Ustates.size(), VI(numOfMoves, -1));
    previousState = vector<vector<int>>(Ustates.size(), VI(numOfMoves, -1));
    for (auto it = EdgeLabel.begin(); it != EdgeLabel.end(); ++it) {
        for (int move : it->second) {
            nextState[(it->first).first][move] = (it->first).second;
            previousState[(it->first).second][move] = (it->first).first;
        }
    }
}

void delayedShield::initializeStrategy_rec(Strategy& Xi, VI& sigmavec, const int pos, const int state) {
    if (pos == sigmavec.size()) {
        Xi[{state, sigmavec}] = vector<bool>(numOfMoves, false);
        return;
    }
    for (int i = 0; i < sigmavec.size(); ++i) {
        if (S0moves[i]) {
            sigmavec[pos] = i;
            initializeStrategy_rec(Xi, sigmavec, pos+1, state);
        }
    }
}

Strategy delayedShield::initializeStrategy(int local_delta) {
    Strategy Xi;
    for (int s=0;s<S0states.size(); ++s) {
        if((S0states[s] and local_delta%2==0) or (S1states[s] and local_delta%2==1)) {
            VI sigmavec(local_delta/2, 0);
            initializeStrategy_rec(Xi, sigmavec, 0, s);
        }
    }
    bool allsome = true;
    for (auto xx : Xi) {
        allsome = allsome and !xx.second.empty();
        if (xx.second.empty()) {
            cout << xx.first.first << ", {";
            print_vector(xx.first.second);
            cout << "}\n";
        }
    }
    assert(allsome);
    return Xi;
}

void delayedShield::initAllPha_rec(VI& vec, int pos) {
    int i = vec.size();
    if (pos >= i) {
        AllHistoryStates[i].push_back(vec);
        return;
    }
    for (int j = 0; j < numOfMoves; ++j) {
        if (S0moves[j]) {
            vec[pos] = j;
            initAllPha_rec(vec, pos + 1);
        }
    }
}

void delayedShield::initializeAllPha() {
    for (int i=0; i < AllHistoryStates.size(); ++i) {
        VI vec(i);
        initAllPha_rec(vec, 0);
    }
}

void delayedShield::initializeS0moves() {
    S0moves = vector<bool>(numOfMoves, false);
    for (int i = 0; i < stateNames.size(); ++i) {
        if (S0states[i]) {
            for (int outvertx : Graph[i].second) {
                for (int move : EdgeLabel[{i,outvertx}])
                    S0moves[move] = true;
            }
        }
    }
}

/// Boolean vector part

bool delayedShield::vbool_is_empty(const vector<bool>& v) {
    if (v.empty()) return true;
    for(bool val : v) {
        if (val) return false;
    }
    return true;
}


int delayedShield::vbool_pop(vector<bool>& v) {
    int poper = -1;
    for (int i =0; i < v.size(); ++i) {
        if (v[i]) {
            poper = i;
            v[i] = false;
            i = v.size();
        }
    }
    return poper;
}

void delayedShield::vbool_insert(vector<bool>& v, const VI& topush) {
    for (int i = 0; i < topush.size(); ++i)
        v[topush[i]] = true;
    return;
}

void delayedShield::vbool_insert(vector<bool>& v, int topush) {
    v[topush] = true;
    return;
}

void delayedShield::vbool_delete(vector<bool>& v, int topush) {
    v[topush] = false;
}

void delayedShield::vbool_delete(vector<bool>& v, const VI& topush) {
    for (int i = 0; i < topush.size(); ++i)
        vbool_delete(v, i);
}

vector<bool> delayedShield::vbool_intersect(const vector<bool>& u, const vector<bool>& v) {
    assert(u.size() == v.size() or u.empty() != v.empty());
    vector<bool> result(max(u.size(), v.size()), false);
    if (u.empty() or v.empty()) return result;
    for (int i = 0; i < u.size(); ++i)
        result[i] = u[i] and v[i];
    return result;
}