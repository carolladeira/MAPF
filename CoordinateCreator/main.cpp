//
//  main.cpp
//  CoordinateCreator
//
//  Created by Nick Zerbel on 1/12/18.
//  Copyright © 2018 Nicholas Zerbel. All rights reserved.
//

#include <iostream>
#include "agent.hpp"

using namespace std;

int main() {
    srand(time(NULL));
    
    multi_agent m;
    
    int max_agents = 21;
    m.x_dim = 8;
    m.y_dim = 8;
    m.n_configs = 30;
    m.create_config_list(max_agents);
    
    return 0;
}
