
#include "tree.hpp"

void multi_tree::create_level(int agn) {
    level l;
    ag_tree.at(agn).tree_vec.push_back(l); //Add a new level to point agn's tree
}


void multi_tree::create_root_node(int lev, int pos, double agx, double agy, int ag_num, int node_num, int p_num, int a,
                                  int agente, int n_agentes) {
    node n;
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.push_back(n);

    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes.resize(n_agentes);
    for (int i = 0; i < ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes.size(); i++) {
        ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes[i].agent_x = -1;
        ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes[i].agent_y = -1;
    }
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).a_number = agente; //Each node stores the point's identifier
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).x = agx; //X-coordinate of a state mapped in the tree
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).y = agy; //Y-Coordinate of a state mapped in the tree
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).n_number = node_num; //Node number is used for tree navigation
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).p_number = p_num; //Parent number is used for tree navigation
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).UCB1 = 0; //Q_UCB1 value
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).q_node = 0; //Q-value
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).action = a; //Action identifier
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).visit_count = 1; //Number of times this state has been visited

   // std::cout << "Create node " << node_num << " Agente: " << ag_num << " level " << lev << " pos " << pos << " | "<< agx << "," << agy << " parent number " << p_num << endl;

}

void multi_tree::create_node(int lev, int pos, double agx, double agy, int ag_num, int node_num, int p_num, int a,
                             int agente) {
    node n;
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.push_back(n);
    for (int p = 0; p < ag_tree.at(ag_num).tree_vec.at(lev - 1).level_vec.size(); p++) {
        if (ag_tree.at(ag_num).tree_vec.at(lev - 1).level_vec.at(p).n_number == p_num) {
            ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes = ag_tree.at(ag_num).tree_vec.at(
                    lev - 1).level_vec.at(p).pos_agentes;
            break;
        }
    }

    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).a_number = agente; //Each node stores the point's identifier
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).x = agx; //X-coordinate of a state mapped in the tree
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).y = agy; //Y-Coordinate of a state mapped in the tree
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).n_number = node_num; //Node number is used for tree navigation
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).p_number = p_num; //Parent number is used for tree navigation
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).UCB1 = 0; //Q_UCB1 value
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).q_node = 0; //Q-value
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).action = a; //Action identifier
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).visit_count = 1; //Number of times this state has been visited

    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes[agente].agent_x = agx;
    ag_tree.at(ag_num).tree_vec.at(lev).level_vec.at(pos).pos_agentes[agente].agent_y = agy;
    //std::cout << "Create node " << node_num << " Agente: " << ag_num << " level " << lev << " pos " << pos << " | "<< agx << "," << agy << " parent number " << p_num << endl;

}

void multi_tree::create_tree() {
    tree t;
    ag_tree.push_back(t);

}

void multi_tree::print_tree(int agn) {
   // cout << " ==========================================" << endl;

    for (int lev = 0; lev < ag_tree.at(agn).tree_vec.size(); lev++) {
        cout << "------Level " << lev << endl;

        for (int en = 0;
             en < ag_tree.at(agn).tree_vec.at(lev).level_vec.size(); en++) { //Rollout all newly expanded nodes
            int n_number = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).n_number;
            double ucb = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).UCB1;
            int n_visit = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).visit_count;
            double q_value = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).q_node;
            int agx = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).x;
            int agy = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).y;
            int p_num = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).p_number;
            int agente = ag_tree.at(agn).tree_vec.at(lev).level_vec.at(en).a_number;
            std::cout << "Node " << n_number << " pos " << en <<" Agente "<<agente<< " UCB1 " << ucb << " N-visit " << n_visit
                      << " q-value " << q_value << " | " << agx << "," << agy << " parent  " << p_num << endl;

        }
    }


}
