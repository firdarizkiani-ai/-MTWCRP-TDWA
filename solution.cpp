//Solution
#include "class.h"
#include <iostream>
#include <cmath>
#include <vector>
using std::cout; using std::endl;

Solution::Solution(DataProb& data_, MasterProblem& master_)
    : master(master_), data(data_)
{
    // init empty routes with sentinel depot 0..0 per trip
    for (int r = 0; r < data.numTruck; ++r) {
        PairVector1D temp;
        for (int t = 0; t < data.numTripMax + 1; ++t) temp.push_back({ 0,0 });
        allroute.push_back(temp);
    }

    num_visited = IntVector1D(data.numVertex, 0);
    used_trip = IntVector1D(data.numTruck, 0);
    usedTruck = 0;

    x_values = BoolVector4D(
        data.numVertex,
        BoolVector3D(data.numVertex,
            BoolVector2D(data.numTruck,
                BoolVector1D(data.numTripMax, false))));

    y_values = BoolVector4D(
        data.numVertex,
        BoolVector3D(data.numTruck,
            BoolVector2D(data.numFreqMax,
                BoolVector1D(data.numTripMax, false))));

    remain_values = DoubleVector1D(data.numVertex, 0.0);
    for (int i = 0; i < data.numVertex; ++i)
        remain_values[i] = data.initial[i] + data.tmax * data.demandrate[i];

    objfunc = data.BigM;
    isFeasible = false;
    isFeasible_relaxed = false;
}

Solution::~Solution() {}

void Solution::update_related_info()
{
    // count used trips per truck (depot-to-depot gaps)
    used_trip = IntVector1D(data.numTruck, data.numTripMax);
    for (int r = 0; r < (int)allroute.size(); ++r)
        for (int i = 0; i + 1 < (int)allroute[r].size(); ++i)
            if (allroute[r][i].first == 0 && allroute[r][i + 1].first == 0)
                used_trip[r]--;

    usedTruck = 0;
    for (int r = 0; r < (int)allroute.size(); ++r)
        if (used_trip[r] > 0) ++usedTruck;

    num_visited = IntVector1D(data.numVertex, 0);
    x_values = BoolVector4D(
        data.numVertex,
        BoolVector3D(data.numVertex,
            BoolVector2D(data.numTruck,
                BoolVector1D(data.numTripMax, false))));
    y_values = BoolVector4D(
        data.numVertex,
        BoolVector3D(data.numTruck,
            BoolVector2D(data.numFreqMax,
                BoolVector1D(data.numTripMax, false))));

    // build x/y from routes
    for (int r_index = 0; r_index < (int)allroute.size(); ++r_index) {
        int t_index = 0;
        for (int i_index = 0; i_index + 1 < (int)allroute[r_index].size(); ++i_index) {
            int nodei = allroute[r_index][i_index].first;
            int nodej = allroute[r_index][i_index + 1].first;
            x_values[nodei][nodej][r_index][t_index] = 1;
            if (nodej == 0) {
                t_index++; // guard overflow
            }
            else {
                num_visited[nodej]++;
                int f_nodej = allroute[r_index][i_index + 1].second; // frequency label from route
                //if (f_nodej < 1) f_nodej = 1;
                //if (f_nodej >= data.numFreqMax) f_nodej = data.numFreqMax - 1; // keep in range
                y_values[nodej][r_index][f_nodej][t_index] = 1;
            }
        }
    }

    /*
    // --- DEDUPE: ensure at most one (k,t) is true for each (i,f) ---
    for (int i = 1; i < data.numVertex; ++i) {
        for (int f = 1; f < data.numFreqMax; ++f) {
            bool kept = false;
            for (int k = 0; k < data.numTruck; ++k) {
                for (int t = 0; t < data.numTripMax; ++t) {
                    if (y_values[i][k][f][t]) {
                        if (!kept) { kept = true; }  // keep the first found
                        else { y_values[i][k][f][t] = false; } // drop duplicates
                    }
                }
            }
        }
    }
    */
}

void Solution::solve_masterproblem()
{
    // remove previous dynamic constraints
    for (auto& c : master.changingConstraints) master.model.remove(c);
    master.changingConstraints.clear();

    // -------- constr (7): depa timing along arcs in x --------
    for (int i = 0; i < data.numVertex; ++i) {
        for (int j = 1; j < data.numVertex; ++j) if (i != j) {
            for (int k = 0; k < data.numTruck; ++k) {
                for (int t = 0; t < data.numTripMax; ++t)
                    if (x_values[i][j][k][t] == 1)
                    {
                        GRBConstr c = master.model.addConstr(
                            master.depa[j][k][t] - master.depa[i][k][t] - data.dist[i][j] >= 0.0
                        );
                        master.changingConstraints.push_back(c);
                    }
            }
        }
    }

    // -------- constr (8): synchronize next trip when returning to depot --------
    for (int i = 1; i < data.numVertex; ++i) {
        for (int k = 0; k < data.numTruck; ++k) {
            for (int t = 1; t < data.numTripMax; ++t)
                if (x_values[i][0][k][t - 1] == 1)
                {
                    GRBConstr c = master.model.addConstr(
                        master.depa[0][k][t] - master.depa[i][k][t - 1] - data.dist[i][0] >= 0.0
                    );
                    master.changingConstraints.push_back(c);
                }
        }
    }

    // -------- constr (9): vehicle capacity per (k,t) --------
    for (int k = 0; k < data.numTruck; ++k) {
        for (int t = 0; t < data.numTripMax; ++t) {
            GRBLinExpr load = 0.0;
            for (int i = 1; i < data.numVertex; ++i)
                for (int f = 1; f < data.numFreqMax; ++f)
                    load += y_values[i][k][f][t] * master.pickq[i][f];
            GRBConstr c = master.model.addConstr(load <= data.vehicle_capacity);
            master.changingConstraints.push_back(c);
        }
    }

    // -------- constr (10): link ptime and depa for each (i,f) --------
    for (int i = 1; i < data.numVertex; ++i) {
        for (int f = 1; f < data.numFreqMax; ++f) {
            int times = 0;
            GRBLinExpr sumDepa = 0.0;
            for (int k = 0; k < data.numTruck; ++k)
                for (int t = 0; t < data.numTripMax; ++t)
                    if (y_values[i][k][f][t]) { times++; sumDepa += master.depa[i][k][t]; }
            GRBConstr c = master.model.addConstr(
                master.ptime[i][f] - (1 - times) * master.ptime[i][f - 1] - sumDepa == 0.0
            );
            master.changingConstraints.push_back(c);
        }
    }

    /*
    // ---- (A) monotone timing across frequencies: ptime[i][f] >= ptime[i][f-1] ----
    for (int i = 1; i < data.numVertex; ++i) {
        for (int f = 1; f < data.numFreqMax; ++f) {
            master.changingConstraints.push_back(
                master.model.addConstr(master.ptime[i][f] - master.ptime[i][f - 1] >= 0.0)
            );
        }
    }

    // ---- (B) bind ptime[i][f] to exactly ONE chosen (k,t), or carry over if none ----
    for (int i = 1; i < data.numVertex; ++i) {
        for (int f = 1; f < data.numFreqMax; ++f) {
            int chosen_k = -1, chosen_t = -1;
            for (int k = 0; k < data.numTruck && chosen_k == -1; ++k)
                for (int t = 0; t < data.numTripMax && chosen_t == -1; ++t)
                    if (y_values[i][k][f][t]) { chosen_k = k; chosen_t = t; }

            if (chosen_k != -1) {
                // single source of truth for (i,f)
                master.changingConstraints.push_back(
                    master.model.addConstr(master.ptime[i][f] - master.depa[i][chosen_k][chosen_t] == 0.0)
                );
            }
            else {
                // no visit for this frequency -> same as previous frequency
                master.changingConstraints.push_back(
                    master.model.addConstr(master.ptime[i][f] - master.ptime[i][f - 1] == 0.0)
                );
            }
        }
    }
    */

    // -------- first-visit pickup --------
    for (int i = 1; i < data.numVertex; ++i) {
        int firstCount = 0;
        for (int k = 0; k < data.numTruck; ++k)
            for (int t = 0; t < data.numTripMax; ++t)
                if (y_values[i][k][1][t]) firstCount++;
        GRBConstr c = master.model.addConstr(
            master.pickq[i][1] - master.ptime[i][1] * data.demandrate[i] - firstCount * data.initial[i] == 0.0
        );
        master.changingConstraints.push_back(c);
    }

    /*
    // ---- (C) first-visit pickup: count AFTER dedupe and clamp to {0,1} ----
    for (int i = 1; i < data.numVertex; ++i) {
        int firstCount = 0;
        for (int k = 0; k < data.numTruck; ++k)
            for (int t = 0; t < data.numTripMax; ++t)
                if (y_values[i][k][1][t]) firstCount++;
        if (firstCount > 1) firstCount = 1;

        master.changingConstraints.push_back(
            master.model.addConstr(
                master.pickq[i][1] - master.ptime[i][1] * data.demandrate[i] - firstCount * data.initial[i] == 0.0
            )
        );
    }

    // ---- (D) anchor depot departure for first trips ----
    for (int k = 0; k < data.numTruck; ++k) {
        master.changingConstraints.push_back(
            master.model.addConstr(master.depa[0][k][0] == 0.0)
        );
    }
    */

    master.model.update();
    master.model.optimize();

    const int status = master.model.get(GRB_IntAttr_Status);
    isFeasible_relaxed = (status == GRB_OPTIMAL || status == GRB_SUBOPTIMAL);
    isFeasible = true;

    if (isFeasible_relaxed) {
        for (int i = 1; i < data.numVertex; ++i) {
            double remain = master.tau_e[i].get(GRB_DoubleAttr_X);
            if (remain >= 0.0) remain_values[i] = remain;
            if (remain > data.upper_limit_waste + 1e-6) { isFeasible = false; break; }
        }

        double penaltyObj = master.model.get(GRB_DoubleAttr_ObjVal);

        double travelCost = 0.0;
        for (int i = 0; i < data.numVertex; ++i)
            for (int j = 0; j < data.numVertex; ++j)
                for (int k = 0; k < data.numTruck; ++k)
                    for (int t = 0; t < data.numTripMax; ++t)
                        if (i != j)
                        {
                            travelCost += data.dist[i][j] * x_values[i][j][k][t];
                        }

        int vehiclesUsed = 0;
        for (int k = 0; k < data.numTruck; ++k)
            if (used_trip[k] > 0) vehiclesUsed++;

        //objfunc = penaltyObj + travelCost + (vehiclesUsed * data.fixedCost);
        //objfunc = penaltyObj;
        //objfunc = travelCost;
        //objfunc = (vehiclesUsed * data.fixedCost);

        objfunc = travelCost + (vehiclesUsed * data.fixedCost);

        //Activated when calculate the penalty cost
        /*
        double sumTau = 0.0;
        for (int i = 1; i < data.numVertex; ++i) {     // skip depot 0
            double tau = master.exceed_waste[i].get(GRB_DoubleAttr_X);
            remain_values[i] = tau;
            sumTau += tau;
        }
        objfunc += sumTau * data.penalty; //+ travelCost + (vehiclesUsed * data.fixedCost);
        // */
    }
    else {
        isFeasible = false;
        objfunc = data.BigM;
    }
}

void Solution::insert_a_bin(int route_index, int pos_index, pair<int, int> bin, const bool skip_solve_masterproblem)
{
    if (route_index >= data.numTruck)
    {
        cout << "define route_index is wrong!"; system("pause");
    }
    else if (pos_index >= allroute[route_index].size())
    {
        cout << "define pos_index is wrong!";

    }
    else
    {
        /* update related info */

        if (used_trip[route_index] == 0)
        {
            usedTruck++;
        }

        int pre_bin = allroute[route_index][pos_index - 1].first;
        int suc_bin = allroute[route_index][pos_index].first;

        if (pre_bin == 0 and suc_bin == 0)
        {
            used_trip[route_index]++;
        }

        int trip_index = 0;
        for (int i = 1; i < pos_index; i++)
        {
            if (allroute[route_index][i].first == 0)
            {
                trip_index++;
            }
        }

        num_visited[bin.first]++;
        x_values[pre_bin][suc_bin][route_index][trip_index] = 0;
        x_values[pre_bin][bin.first][route_index][trip_index] = 1;
        x_values[bin.first][suc_bin][route_index][trip_index] = 1;
        y_values[bin.first][route_index][bin.second][trip_index] = 1;
        allroute[route_index].insert(allroute[route_index].begin() + pos_index, bin);
        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();
        }
    }
}

void Solution::insert_a_bin2(int route_index, int pos_index, pair<int, int> bin, const bool skip_solve_masterproblem)
{
    if (route_index >= data.numTruck)
    {
        cout << "define route_index is wrong!"; system("pause");
    }
    else if (pos_index >= allroute[route_index].size())
    {
        cout << "define pos_index is wrong!";

    }
    else
    {
        /* update related info */

        if (used_trip[route_index] == 0)
        {
            usedTruck++;
        }

        int pre_bin = allroute[route_index][pos_index - 1].first;
        int suc_bin = allroute[route_index][pos_index].first;

        if (pre_bin == 0 and suc_bin == 0)
        {
            used_trip[route_index]++;
        }

        int trip_index = 0;
        for (int i = 1; i < pos_index; i++)
        {
            if (allroute[route_index][i].first == 0)
            {
                trip_index++;
            }
        }

        num_visited[bin.first]++;
        x_values[pre_bin][suc_bin][route_index][trip_index] = 0;
        x_values[pre_bin][bin.first][route_index][trip_index] = 1;
        x_values[bin.first][suc_bin][route_index][trip_index] = 1;
        y_values[bin.first][route_index][bin.second][trip_index] = 1;
        allroute[route_index].insert(allroute[route_index].begin() + pos_index, bin);

        /*
        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();
        }
        */
    }
}

void Solution::remove_a_bin(int route_index, int pos_index, const bool skip_solve_masterproblem)
{
    if (allroute[route_index][pos_index].first == 0)
    {
        cout << "should not remove depot (0)!\n"; system("pause");
    }
    else
    {
        int pre_bin = allroute[route_index][pos_index - 1].first;
        int suc_bin = allroute[route_index][pos_index + 1].first;
        if (pre_bin == 0 and suc_bin == 0)
        {
            used_trip[route_index]--;
        }
        if (used_trip[route_index] == 0)
        {
            usedTruck--;
        }

        int trip_index = 0;
        for (int i = 1; i < pos_index; i++)
        {
            if (allroute[route_index][i].first == 0)
            {
                trip_index++;
            }
        }

        num_visited[allroute[route_index][pos_index].first]--;
        x_values[pre_bin][suc_bin][route_index][trip_index] = 1;
        x_values[pre_bin][allroute[route_index][pos_index].first][route_index][trip_index] = 0;
        x_values[allroute[route_index][pos_index].first][suc_bin][route_index][trip_index] = 0;
        y_values[allroute[route_index][pos_index].first][route_index][allroute[route_index][pos_index].second][trip_index] = 0;

        allroute[route_index].erase(allroute[route_index].begin() + pos_index);		// remove the bin

        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();						// solve and update the solution
        }
    }
}

void Solution::remove_a_bin2(int route_index, int pos_index, const bool skip_solve_masterproblem)
{
    if (allroute[route_index][pos_index].first == 0)
    {
        cout << "should not remove depot (0)!\n"; system("pause");
    }
    else
    {
        int pre_bin = allroute[route_index][pos_index - 1].first;
        int suc_bin = allroute[route_index][pos_index + 1].first;
        if (pre_bin == 0 and suc_bin == 0)
        {
            used_trip[route_index]--;
        }
        if (used_trip[route_index] == 0)
        {
            usedTruck--;
        }

        int trip_index = 0;
        for (int i = 1; i < pos_index; i++)
        {
            if (allroute[route_index][i].first == 0)
            {
                trip_index++;
            }
        }

        num_visited[allroute[route_index][pos_index].first]--;
        x_values[pre_bin][suc_bin][route_index][trip_index] = 1;
        x_values[pre_bin][allroute[route_index][pos_index].first][route_index][trip_index] = 0;
        x_values[allroute[route_index][pos_index].first][suc_bin][route_index][trip_index] = 0;
        y_values[allroute[route_index][pos_index].first][route_index][allroute[route_index][pos_index].second][trip_index] = 0;

        allroute[route_index].erase(allroute[route_index].begin() + pos_index);		// remove the bin

        /*
        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();						// solve and update the solution
        }
        */
    }
}

void Solution::swap_two_bin(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem)
{
    if (allroute[route1_index][pos1_index].first == 0 or allroute[route2_index][pos2_index].first == 0)
    {
        cout << "should not swap the depot (0)!\n";
    }
    else
    {
        int prebin1 = allroute[route1_index][pos1_index - 1].first;
        int bin1 = allroute[route1_index][pos1_index].first;
        int freq1 = allroute[route1_index][pos1_index].second;
        int sucbin1 = allroute[route1_index][pos1_index + 1].first;
        int bin2 = allroute[route2_index][pos2_index].first;
        int freq2 = allroute[route2_index][pos2_index].second;
        int prebin2 = allroute[route2_index][pos2_index - 1].first;
        int sucbin2 = allroute[route2_index][pos2_index + 1].first;
        int trip1_index = 0;
        int trip2_index = 0;
        for (int i = 1; i < pos1_index; i++)
        {
            if (allroute[route1_index][i].first == 0)
            {
                trip1_index++;
            }
        }
        for (int i = 1; i < pos2_index; i++)
        {
            if (allroute[route2_index][i].first == 0)
            {
                trip2_index++;
            }
        }

        // check the case that node1 and node2 are two consecutive nodes
        if (route1_index == route2_index and (pos1_index == pos2_index + 1 or pos2_index == pos1_index + 1) and pos1_index != 0 and pos2_index != 0)
        {
            // case 1: pos1 is in front of pos2
            if (pos1_index + 1 == pos2_index)
            {
                x_values[prebin1][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][bin2][route1_index][trip1_index] = 0;
                x_values[bin2][sucbin2][route1_index][trip1_index] = 0;

                x_values[prebin1][bin2][route1_index][trip1_index] = 1;
                x_values[bin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][sucbin2][route1_index][trip1_index] = 1;

                // should be the same trip --> no need to update y_values
            }
            // case 2: pos1 is behind pos2
            else if (pos2_index + 1 == pos1_index)
            {
                x_values[prebin2][bin2][route1_index][trip1_index] = 0;
                x_values[bin2][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][sucbin1][route1_index][trip1_index] = 0;

                x_values[prebin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][bin2][route1_index][trip1_index] = 1;
                x_values[bin2][sucbin1][route1_index][trip1_index] = 1;

                // should be the same trip --> no need to update y_values
            }
            else
            {
                cout << "this case has been defined";
            }
        }
        else
        {
            //print_solution();
            //cout << "route1: " << route1_index << " pos1:" << pos1_index << " bin1:" << bin1 << "_" << freq1 << endl;
            //cout << "route2: " << route2_index << " pos2:" << pos2_index << " bin2:" << bin2 << "_" << freq2 << endl;

            // Remove previous x_par
            x_values[prebin1][bin1][route1_index][trip1_index] = 0;
            x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
            x_values[prebin2][bin2][route2_index][trip2_index] = 0;
            x_values[bin2][sucbin2][route2_index][trip2_index] = 0;
            // Update new x_par after swap
            x_values[prebin1][bin2][route1_index][trip1_index] = 1;
            x_values[bin2][sucbin1][route1_index][trip1_index] = 1;
            x_values[prebin2][bin1][route2_index][trip2_index] = 1;
            x_values[bin1][sucbin2][route2_index][trip2_index] = 1;
            // Revised previous y_par (remove the previous bin in the route)
            y_values[bin1][route1_index][freq1][trip1_index] = 0;
            y_values[bin2][route2_index][freq2][trip2_index] = 0;
            // Add new y_par after swap (bin and freq will not change)
            y_values[bin2][route1_index][freq2][trip1_index] = 1;
            y_values[bin1][route2_index][freq1][trip2_index] = 1;
        }

        //update_related_info();
        int temp;
        temp = allroute[route1_index][pos1_index].first;
        allroute[route1_index][pos1_index].first = allroute[route2_index][pos2_index].first;
        allroute[route2_index][pos2_index].first = temp;
        temp = allroute[route1_index][pos1_index].second;
        allroute[route1_index][pos1_index].second = allroute[route2_index][pos2_index].second;
        allroute[route2_index][pos2_index].second = temp;


        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();						// solve and update the solution
        }
        //print_routes();
        //system("pause");
    }
}

void Solution::swap_two_bin2(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem)
{
    if (allroute[route1_index][pos1_index].first == 0 or allroute[route2_index][pos2_index].first == 0)
    {
        cout << "should not swap the depot (0)!\n";
    }
    else
    {
        int prebin1 = allroute[route1_index][pos1_index - 1].first;
        int bin1 = allroute[route1_index][pos1_index].first;
        int freq1 = allroute[route1_index][pos1_index].second;
        int sucbin1 = allroute[route1_index][pos1_index + 1].first;
        int bin2 = allroute[route2_index][pos2_index].first;
        int freq2 = allroute[route2_index][pos2_index].second;
        int prebin2 = allroute[route2_index][pos2_index - 1].first;
        int sucbin2 = allroute[route2_index][pos2_index + 1].first;
        int trip1_index = 0;
        int trip2_index = 0;
        for (int i = 1; i < pos1_index; i++)
        {
            if (allroute[route1_index][i].first == 0)
            {
                trip1_index++;
            }
        }
        for (int i = 1; i < pos2_index; i++)
        {
            if (allroute[route2_index][i].first == 0)
            {
                trip2_index++;
            }
        }

        // check the case that node1 and node2 are two consecutive nodes
        if (route1_index == route2_index and (pos1_index == pos2_index + 1 or pos2_index == pos1_index + 1) and pos1_index != 0 and pos2_index != 0)
        {
            // case 1: pos1 is in front of pos2
            if (pos1_index + 1 == pos2_index)
            {
                x_values[prebin1][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][bin2][route1_index][trip1_index] = 0;
                x_values[bin2][sucbin2][route1_index][trip1_index] = 0;

                x_values[prebin1][bin2][route1_index][trip1_index] = 1;
                x_values[bin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][sucbin2][route1_index][trip1_index] = 1;

                // should be the same trip --> no need to update y_values
            }
            // case 2: pos1 is behind pos2
            else if (pos2_index + 1 == pos1_index)
            {
                x_values[prebin2][bin2][route1_index][trip1_index] = 0;
                x_values[bin2][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][sucbin1][route1_index][trip1_index] = 0;

                x_values[prebin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][bin2][route1_index][trip1_index] = 1;
                x_values[bin2][sucbin1][route1_index][trip1_index] = 1;

                // should be the same trip --> no need to update y_values
            }
            else
            {
                cout << "this case has been defined";
            }
        }
        else
        {
            //print_solution();
            //cout << "route1: " << route1_index << " pos1:" << pos1_index << " bin1:" << bin1 << "_" << freq1 << endl;
            //cout << "route2: " << route2_index << " pos2:" << pos2_index << " bin2:" << bin2 << "_" << freq2 << endl;

            // Remove previous x_par
            x_values[prebin1][bin1][route1_index][trip1_index] = 0;
            x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
            x_values[prebin2][bin2][route2_index][trip2_index] = 0;
            x_values[bin2][sucbin2][route2_index][trip2_index] = 0;
            // Update new x_par after swap
            x_values[prebin1][bin2][route1_index][trip1_index] = 1;
            x_values[bin2][sucbin1][route1_index][trip1_index] = 1;
            x_values[prebin2][bin1][route2_index][trip2_index] = 1;
            x_values[bin1][sucbin2][route2_index][trip2_index] = 1;
            // Revised previous y_par (remove the previous bin in the route)
            y_values[bin1][route1_index][freq1][trip1_index] = 0;
            y_values[bin2][route2_index][freq2][trip2_index] = 0;
            // Add new y_par after swap (bin and freq will not change)
            y_values[bin2][route1_index][freq2][trip1_index] = 1;
            y_values[bin1][route2_index][freq1][trip2_index] = 1;
        }

        //update_related_info();
        int temp;
        temp = allroute[route1_index][pos1_index].first;
        allroute[route1_index][pos1_index].first = allroute[route2_index][pos2_index].first;
        allroute[route2_index][pos2_index].first = temp;
        temp = allroute[route1_index][pos1_index].second;
        allroute[route1_index][pos1_index].second = allroute[route2_index][pos2_index].second;
        allroute[route2_index][pos2_index].second = temp;

        /*
        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();						// solve and update the solution
        }
        */
        //print_routes();
        //system("pause");
    }
}

void Solution::relocate_a_bin(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem)
{
    if (allroute[route1_index][pos1_index].first == 0)
    {
        cout << "route_1:" << route1_index << " check pos i:" << pos1_index << " route_2:" << route2_index << " check pos j:" << pos2_index << endl;
        cout << "should not relocate the depot (0)!\n";
    }
    else
    {
        int prebin1 = allroute[route1_index][pos1_index - 1].first;
        int bin1 = allroute[route1_index][pos1_index].first;
        int sucbin1 = allroute[route1_index][pos1_index + 1].first;
        int prebin2 = allroute[route2_index][pos2_index - 1].first;
        int bin2 = allroute[route2_index][pos2_index].first;
        int sucbin2 = allroute[route2_index][pos2_index + 1].first;
        int trip1_index = 0;
        int trip2_index = 0;
        for (int i = 1; i < pos1_index; i++)
        {
            if (allroute[route1_index][i].first == 0)
            {
                trip1_index++;
            }
        }
        for (int i = 1; i < pos2_index; i++)
        {
            if (allroute[route2_index][i].first == 0)
            {
                trip2_index++;
            }
        }

        // check the case that node1 and node2 are two consecutive nodes (RelocateIntra)
        if (route1_index == route2_index and trip1_index == trip2_index)
        {
            // case 1: pos1 is in front of pos2
            if (pos1_index + 1 == pos2_index)
            {
                x_values[prebin1][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][bin2][route1_index][trip1_index] = 0;
                x_values[bin2][sucbin2][route2_index][trip2_index] = 0;

                x_values[prebin1][bin2][route1_index][trip1_index] = 1;
                x_values[bin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][sucbin2][route1_index][trip1_index] = 1;

                // should be the same trip --> no need to update y_values
            }
            // case 2: pos1 is behind pos2
            else if (pos2_index + 1 == pos1_index)
            {
                // Special case for RelocateInter
                if (bin2 == 0)
                {
                    x_values[bin2][bin1][route1_index][trip1_index] = 0;
                    x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
                    x_values[prebin2][bin2][route1_index][trip2_index - 1] = 0;

                    x_values[prebin2][bin1][route1_index][trip2_index - 1] = 1;
                    x_values[bin1][bin2][route1_index][trip2_index - 1] = 1;
                    x_values[bin2][sucbin1][route1_index][trip1_index] = 1;

                    y_values[bin1][route1_index][allroute[route1_index][pos1_index].second][trip1_index] = 0;
                    // Add new y_par after relocate (bin and freq will not change)
                    y_values[bin1][route2_index][allroute[route1_index][pos1_index].second][trip2_index - 1] = 1;


                }
                else
                {
                    x_values[prebin2][bin2][route1_index][trip1_index] = 0;
                    x_values[bin2][bin1][route1_index][trip1_index] = 0;
                    x_values[bin1][sucbin1][route1_index][trip1_index] = 0;

                    x_values[prebin2][bin1][route1_index][trip1_index] = 1;
                    x_values[bin1][bin2][route1_index][trip1_index] = 1;
                    x_values[bin2][sucbin1][route1_index][trip1_index] = 1;
                }
            }
            // case 3: pos1 in front of pos2 but not near to
            else if (pos1_index < pos2_index - 1)
            {
                x_values[prebin1][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
                x_values[bin2][sucbin2][route2_index][trip2_index] = 0;

                x_values[prebin1][sucbin1][route1_index][trip1_index] = 1;
                x_values[bin2][bin1][route2_index][trip2_index] = 1;
                x_values[bin1][sucbin2][route2_index][trip2_index] = 1;

            }
            // case 3: pos1 behind pos2 but not near to
            else if (pos2_index < pos1_index - 1)
            {
                x_values[prebin1][bin1][route1_index][trip1_index] = 0;
                x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
                x_values[prebin2][bin2][route1_index][trip1_index] = 0;

                x_values[prebin2][bin1][route1_index][trip1_index] = 1;
                x_values[bin1][bin2][route1_index][trip1_index] = 1;
                x_values[prebin1][sucbin1][route1_index][trip1_index] = 1;
            }
        }

        else
        {
            // Remove previous x_par
            x_values[prebin1][bin1][route1_index][trip1_index] = 0;
            x_values[bin1][sucbin1][route1_index][trip1_index] = 0;
            x_values[prebin2][bin2][route2_index][trip2_index] = 0;
            // Update new x_par after relocate
            x_values[prebin1][sucbin1][route1_index][trip1_index] = 1;
            x_values[prebin2][bin1][route2_index][trip2_index] = 1;
            x_values[bin1][bin2][route2_index][trip2_index] = 1;
            // Revised previous y_par (remove the previous bin in the route)
            y_values[bin1][route1_index][allroute[route1_index][pos1_index].second][trip1_index] = 0;
            // Add new y_par after relocate (bin and freq will not change)
            y_values[bin1][route2_index][allroute[route1_index][pos1_index].second][trip2_index] = 1;
        }
        if (route1_index == route2_index and trip1_index != trip2_index and pos1_index < pos2_index)
        {
            pair<int, int> node = allroute[route1_index][pos1_index];
            //cout << "check the erase and insert" << endl;
            allroute[route1_index].erase(allroute[route1_index].begin() + pos1_index);
            //cout << route1_index << "\t" << pos1_index << endl;
            //print_routes();
            allroute[route2_index].insert(allroute[route2_index].begin() + pos2_index - 1, node);
            //cout << route2_index << "\t" << pos2_index << endl;
        }
        else
        {
            pair<int, int> node = allroute[route1_index][pos1_index];
            //cout << "check the erase and insert" << endl;
            allroute[route1_index].erase(allroute[route1_index].begin() + pos1_index);
            //cout << route1_index << "\t" << pos1_index << endl;
            //print_routes();
            allroute[route2_index].insert(allroute[route2_index].begin() + pos2_index, node);
            //cout << route2_index << "\t" << pos2_index << endl;
        }

        if (!skip_solve_masterproblem)
        {
            solve_masterproblem();	// solve and update the solution
        }
        //print_routes();
        //system("pause");
    }
}

void Solution::print_solution()
{
    print_routes();

    cout << "num_visited:\t";
    for (int value : num_visited)
    {
        cout << value << "\t";
    }
    cout << endl;

    cout << "used_trip:\t";
    for (int value : used_trip)
    {
        cout << value << "\t";
    }
    cout << endl;

    cout << "remain values:\t";
    for (double value : remain_values)
    {
        cout << value << "\t";
    }
    cout << endl;

    DoubleVector2D pickq_(
        data.numVertex,
        DoubleVector1D(data.numFreqMax, 0.0)
    );

    /*
    std::cout << "pickq[i][f]:\n";
    for (int i = 1; i < data.numVertex; ++i) {
        std::cout << "i=" << i << ":\t";
        for (int f = 1; f < data.numFreqMax; ++f)
        {
            pickq_[i][f] = master.pickq[i][f].get(GRB_DoubleAttr_X);
            std::cout << pickq_[i][f] << '\t';
        }
        std::cout << '\n';
    }

    DoubleVector1D exceed_waste_(data.numVertex, 0.0);
    cout << "exceed_waste:\t";
    for (int i = 1; i < data.numVertex; i++)
    {
        exceed_waste_[i] = master.exceed_waste[i].get(GRB_DoubleAttr_X);
        cout << exceed_waste_[i] << "\t";
    }
    cout << endl;
    */
}

void Solution::print_routes()
{
    cout << "Objective value: " << objfunc << endl;
    cout << "isFeasible:" << isFeasible << endl;
    cout << "isFeasible_relaxed:" << isFeasible_relaxed << endl;
    cout << "Routes:\n";
    for (int r = 0; r < allroute.size(); r++)
    {
        for (int i = 0; i < allroute[r].size(); i++)
        {
            cout << allroute[r][i].first << "_" << allroute[r][i].second << "\t";
        }
        cout << endl;
    }
}

void Solution::print_xvalues()
{
    cout << "x_values" << endl;
    cout << "i" << "\t" << "j" << "\t" << "k" << "\t" << "t" << endl;
    for (int i = 0; i < x_values.size(); i++)
    {
        for (int j = 0; j < x_values[i].size(); j++)
        {
            for (int k = 0; k < x_values[i][j].size(); k++)
            {
                for (int t = 0; t < x_values[i][j][k].size(); t++)
                {
                    if (x_values[i][j][k][t] == 1)
                    {
                        cout << i << "\t" << j << "\t" << k << "\t" << t << endl;
                    }
                }
            }
        }
    }
}

void Solution::print_yvalues()
{
    cout << "y_values" << endl;
    cout << "i" << "\t" << "k" << "\t" << "f" << "\t" << "t" << endl;
    for (int i = 0; i < y_values.size(); i++)
    {
        for (int k = 0; k < y_values[i].size(); k++)
        {
            for (int f = 0; f < y_values[i][k].size(); f++)
            {
                for (int t = 0; t < y_values[i][k][f].size(); t++)
                {
                    if (y_values[i][k][f][t] == 1)
                    {
                        cout << i << "\t" << k << "\t" << f << "\t" << t << endl;
                    }
                }
            }
        }
    }

}

void Solution::copy_from_sol(Solution* fromSol)
{
    allroute.clear();
    for (int i = 0; i < (int)fromSol->allroute.size(); i++) {
        PairVector1D temp;
        for (int j = 0; j < (int)fromSol->allroute[i].size(); j++)
            temp.push_back(fromSol->allroute[i][j]);
        allroute.push_back(temp);
    }

    for (int i = 0; i < data.numVertex; i++)
        num_visited[i] = fromSol->num_visited[i];

    for (int r = 0; r < data.numTruck; r++)
        used_trip[r] = fromSol->used_trip[r];

    usedTruck = fromSol->usedTruck;

    for (int i = 0; i < data.numVertex; i++)
        for (int j = 0; j < data.numVertex; j++)
            for (int k = 0; k < data.numTruck; k++)
                for (int t = 0; t < data.numTripMax; t++)
                    x_values[i][j][k][t] = fromSol->x_values[i][j][k][t];

    for (int i = 0; i < data.numVertex; i++)
        for (int k = 0; k < data.numTruck; k++)
            for (int f = 0; f < data.numFreqMax; f++)
                for (int t = 0; t < data.numTripMax; t++)
                    y_values[i][k][f][t] = fromSol->y_values[i][k][f][t];

    for (int i = 0; i < data.numVertex; i++)
        remain_values[i] = fromSol->remain_values[i];

    isFeasible_relaxed = fromSol->isFeasible_relaxed;
    isFeasible = fromSol->isFeasible;
    objfunc = fromSol->objfunc;
}

void Solution::freeSolution() {}

void Solution::resetSolution()
{
    for (int r = 0; r < data.numTruck; r++)
    {
        PairVector1D temp = {};
        for (int t = 0; t < data.numTripMax + 1; t++)
        {
            temp.push_back(make_pair(0, 0));
        }
        allroute.push_back(temp);
    }

    num_visited = IntVector1D(data.numVertex, 0);
    used_trip = IntVector1D(data.numTruck, 0);
    usedTruck = 0;


    x_values = BoolVector4D(data.numVertex, BoolVector3D(data.numVertex, BoolVector2D(data.numTruck, BoolVector1D(data.numTripMax, 0))));
    y_values = BoolVector4D(data.numVertex, BoolVector3D(data.numTruck, BoolVector2D(data.numFreqMax, BoolVector1D(data.numTripMax, 0))));
    remain_values = DoubleVector1D(data.numVertex, 0.0);
    for (int i = 0; i < data.numVertex; i++)
    {
        remain_values[i] = data.initial[i] + data.tmax * data.demandrate[i];
    }

    objfunc = data.BigM;
    isFeasible_relaxed = false;
    isFeasible = false;
}
