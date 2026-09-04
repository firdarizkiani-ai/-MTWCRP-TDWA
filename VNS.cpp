#include "class.h"
#include <ctime>
#include <cmath>
#include <unordered_map>

VNS::VNS(DataProb& d, MasterProblem& m) : data(d), master(m)
{
    iSol = new Solution(data, master);
    tSol = new Solution(data, master);
    cSol = new Solution(data, master);
    bSol = new Solution(data, master);
    ttSol = new Solution(data, master);
    CPU = 0.0;
}
VNS::~VNS() {}

void VNS::freeVNS()
{
    iSol->freeSolution(); tSol->freeSolution(); cSol->freeSolution(); bSol->freeSolution();
    delete iSol; delete tSol; delete cSol; delete bSol;
}

void VNS::implement(int eta_nonimp, int eta_cool, double T0, double Tf, double alpha, double beta)
{
    double random, delta;
    double START, END;
    START = clock();

    // initialize an initial solution
    createInitialSolution();

    iSol->solve_masterproblem();
    //cout << "======================INITIAL SOLUTION==================\n";
    //iSol->print_solution();
    //cout << "Processing time: " << (END - START) / CLOCKS_PER_SEC << endl;
    //cout << "========================================================\n";
    //system("pause");

    tSol->copy_from_sol(iSol);
    cSol->copy_from_sol(iSol);
    
    //cout << "======================VNS SOLUTION=======================\n";
    //START = clock();
    // initial parameters: temperature
    int iter_nonimp = 0;
    int iter = 0;
    double T = T0;
    bool foundBest = false;
    double runtime = 0.0;

    // Set a maximum iteration limit to avoid infinite loops
    const int maxIterations = 100;      // Maximum number of iterations
    int stagnation_count = 0;          // Counter for tracking stagnation (no improvement)

    while (T >= Tf and iter_nonimp < eta_nonimp and runtime < 3600 and iter < maxIterations)
    {
        iter++;
        tSol->copy_from_sol(cSol);
        bool isChanged = false;
        int nbMoves = 10;
        int selected_operator = rand() % nbMoves;
        int trial = 10;
        int count = 0;
        do
        {
            count++;
            //cout << "select operator:" << selected_operator << endl;
            if (selected_operator == 0)
            {
                isChanged = swap_intra_random(tSol);
            }
            else if (selected_operator == 1)
            {
                isChanged = relocate_intra_random(tSol);
            }
            else if (selected_operator == 2)
            {
                isChanged = remove_random(tSol);
            }
            else if (selected_operator == 3)
            {
                isChanged = relocate_inter_random(tSol);
            }
            else if (selected_operator == 4)
            {
                isChanged = swap_inter_random(tSol);
            }
            else if (selected_operator == 5)
            {
                isChanged = insert_random(tSol);
            }
            else if (selected_operator == 6)
            {
                isChanged = swap22_intra_random(tSol);
            }
            else if (selected_operator == 7)
            {
                isChanged = swap22_inter_random(tSol);
            }
            else if (selected_operator == 8)
            {
                isChanged = shift20_intra_random(tSol);
            }
            else if (selected_operator == 9)
            {
                isChanged = shift20_inter_random(tSol);
            }
        } while (!isChanged and count < trial);

        local_search(tSol);

        // Ensure feasibility after the move
        if (!tSol->isFeasible)
        {
            stagnation_count++;
            continue;
        }

        // update current solution
        if (tSol->objfunc <= cSol->objfunc - ERROR)
        {
            cSol->copy_from_sol(tSol);
        }
        else
        {
            // accept worse solution
            random = (double)rand() / (RAND_MAX + 1.0);
            delta = tSol->objfunc - cSol->objfunc;
            if (random < exp(-delta / (beta * T)))
            {
                cSol->copy_from_sol(tSol);
            }
        }

        // update best solution
        if (tSol->objfunc < bSol->objfunc - ERROR and tSol->isFeasible)
        {
            bSol->copy_from_sol(tSol);
            iter_nonimp = 0;
            foundBest = true;
        }

        if (iter % eta_cool == 0)
        {
            T *= alpha;
            iter = 0;

            // update best solution
            if (tSol->objfunc < bSol->objfunc - ERROR and tSol->isFeasible)
            {
                bSol->copy_from_sol(tSol);
                iter_nonimp = 0;
                foundBest = true;
            }

            if (!foundBest)
            {
                iter_nonimp++;
            }
        }

        // Check if temperature is still above Tf
        if (T < Tf)
        {
            cout << "Stopping due to temperature reaching Tf" << endl;
            break;
        }

        // If the solution has stagnated (no improvement for a certain number of iterations), stop
        if (stagnation_count > 100)  // No improvement for 100 iterations
        {
            cout << "Stopping due to stagnation after " << stagnation_count << " iterations without improvement." << endl;
            break;
        }

        END = clock();
        runtime = (END - START) / CLOCKS_PER_SEC;
        cout << "Iteration: " << iter << " | Temp: " << T << " | Best Solution: " << bSol->objfunc << endl;
    }


    //cout << "========================================================\n";
    cout << "VNS-math's processing time: " << (END - START) / CLOCKS_PER_SEC << endl;
    cout << "The best solution: " << bSol->objfunc << endl;
    bSol->print_solution();
    //cout << "========================================================\n";
}

void VNS::perturbation(Solution* sol)
{
    // We can use RVND as the main search strategy for perturbation
    // It will search through different neighborhoods
    //RVND(sol);

    //bool isChanged = false;

    // After applying RVND, we can still apply adaptive operator selection for further exploration if necessary
    int nbMoves = 12; // Number of available moves (neighborhoods)
    //int selected_operator = rand() % nbMoves;

    /**/
    double random_prob = (double)rand() / RAND_MAX;
    int selected_operator = 0;

    double success_rate[12];
    double success_count[12];
    double total_trials = 0.0;
    for (int i = 0; i < nbMoves; i++) {
        success_rate[i] = 0.0;  // Success rate for each operator
        success_count[i] = 0.0;  // Success count for each operator
    }

    // Apply perturbation based on the adaptive probability distribution
    double total_success = 0.0;
    for (int i = 0; i < nbMoves; i++) {
        total_success += success_rate[i];
    }

    double probabilities[12];
    for (int i = 0; i < nbMoves; i++) {
        probabilities[i] = success_rate[i] / total_success;  // Normalize probabilities
    }

    double cumulative_prob = 0.0;
    for (int i = 0; i < nbMoves; i++) {
        cumulative_prob += probabilities[i];  // Cumulative probability
        if (random_prob <= cumulative_prob) {
            selected_operator = i;
            break;
        }
    }


    // Apply the selected perturbation operator
    if (selected_operator == 0)
        swap_intra_random(sol);
    else if (selected_operator == 1)
        relocate_intra_random(sol);
    else if (selected_operator == 2)
        swap_inter_random(sol);
    else if (selected_operator == 3)
        relocate_inter_random(sol);
    else if (selected_operator == 4)
        remove_random(sol);
    else if (selected_operator == 5)
        insert_random(sol);
    else if (selected_operator == 6)
        two_opt_intra(sol);  // 2-opt operator
    else if (selected_operator == 7)
        three_opt_intra(sol);  // 3-opt operator

    else if (selected_operator == 8)
        swap22_intra_random(sol);
    else if (selected_operator == 9)
        swap22_inter_random(sol);
    else if (selected_operator == 10)
        shift20_intra_random(sol);
    else if (selected_operator == 11)
        shift20_inter_random(sol);

    /*
    else if (selected_operator == 8)
        swap_intra_random2(sol);
    else if (selected_operator == 9)
        swap_inter_random2(sol);
    */

    /*
    else if (selected_operator == 7)
        swap_intra_random2(sol);
    else if (selected_operator == 8)
        relocate_intra_random2(sol);
    else if (selected_operator == 9)
        swap_inter_random2(sol);
    else if (selected_operator == 10)
        relocate_inter_random2(sol);
    */

    if (sol < tSol)
    {
        tSol->copy_from_sol(sol);
        //success_count[selected_operator]++;
    }
    else
    {
        //success_count[selected_operator]--;
    }
    /*
    total_trials++;
    for (int i = 0; i < nbMoves; i++) {
        success_rate[i] = success_count[i] / total_trials;  // Normalize the success rate
    }
    */
}

void VNS::RVND(Solution* sol)
{
    // Number of neighborhoods (e.g., Swap, Relocate, Insert, Remove, 2-opt)
    int nbMoves = 7;
    bool isImproved = false;

    // RVND iterates through multiple neighborhoods
    for (int neighborhood = 0; neighborhood < nbMoves; ++neighborhood)
    {
        switch (neighborhood)
        {
        case 0: // Swap Intra
            isImproved = swap_intra_random(sol);
            break;
        case 1: // Relocate Intra
            isImproved = relocate_intra_random(sol);
            break;
        case 2: // Swap Inter
            isImproved = swap_inter_random(sol);
            break;
        case 3: // Relocate Inter
            isImproved = relocate_inter_random(sol);
            break;
        case 4: // Remove
            isImproved = remove_random(sol);
            break;
        case 5: // Insert
            isImproved = insert_random(sol);
            break;
        case 6: // 2-opt
            isImproved = two_opt_intra(sol);
            break;
        }

        if (isImproved)
        {
            // If improvement is found, stop exploring other neighborhoods
            break;
        }
    }
}

void VNS::createInitialSolution()
{
    int r_index = 0;
    bool havingInsertion = false;
    do
    {
        havingInsertion = false;
        for (int r = 0; r < data.numTruck; r++)
        {
            /* Sorting potential bins should be visited based on the num_visited and remain_values */
            vector<vector<double>> potential_list = {};
            for (int i = 1; i < data.numVertex; i++)
            {
                if (iSol->num_visited[i] < data.numFreqMax - 1 and iSol->remain_values[i] > data.upper_limit_waste)
                {
                    potential_list.push_back({ (double)i, (double)iSol->num_visited[i], iSol->remain_values[i] });
                }
            }
            sort(potential_list.begin(), potential_list.end(), [](const vector<double>& a, const vector<double>& b) {
                if ((int)a[1] != (int)b[1]) return a[1] < b[1];
                else return a[2] > b[2];		});

            if (potential_list.size() == 0)
            {
                break;
            }

            /* Insertion a bin into the route */
            do
            {
                int bin = potential_list[0][0];
                int bin_freq = iSol->num_visited[potential_list[0][0]] + 1;
                pair<int, int> potential_bin = make_pair(bin, bin_freq);

                int insert_index = -1;
                if (iSol->used_trip[r] == 0) { insert_index = 1; }
                else { insert_index = iSol->allroute[r].size() - (data.numTripMax - iSol->used_trip[r] + 1); }
                iSol->insert_a_bin(r, insert_index, potential_bin);
                if (iSol->isFeasible_relaxed)
                {
                    havingInsertion = true;
                    potential_list.erase(potential_list.begin());
                    break;
                }
                else
                {
                    iSol->remove_a_bin(r, insert_index, true);
                    // check with the next trip
                    if (iSol->used_trip[r] < data.numTripMax)
                    {
                        insert_index = iSol->allroute[r].size() - (data.numTripMax - iSol->used_trip[r]);
                        iSol->insert_a_bin(r, insert_index, potential_bin);
                        if (iSol->isFeasible_relaxed)
                        {
                            havingInsertion = true;
                            potential_list.erase(potential_list.begin());
                            break;
                        }
                        else
                        {
                            iSol->remove_a_bin(r, insert_index, true);
                            potential_list.erase(potential_list.begin());
                        }
                    }
                    else
                    {
                        potential_list.erase(potential_list.begin());
                    }
                }
            } while (potential_list.size() != 0);
        }
    } while (!iSol->isFeasible and havingInsertion);


    iSol->solve_masterproblem();
}

// local_search to perform the 7 operators including 2-opt
bool VNS::local_search(Solution* sol)
{
    bool isImprove = false;
    do
    {
        //sol->print_solution();
        isImprove = false;
        if (relocate_inter_fi(sol))
        {
            isImprove = true;
        }

        if (swap_intra_fi(sol))
        {
            isImprove = true;
        }

        if (relocate_intra_fi(sol))
        {
            isImprove = true;
        }

        if (swap_inter_fi(sol))
        {
            isImprove = true;
        }


        if (two_opt_intra(sol))
        {
            isImprove = true;
        }

        if (three_opt_intra(sol))
        {
            isImprove = true;
        }

        if (swap22_intra_fi(sol))
        {
            isImprove = true;
        }

        if (swap22_inter_fi(sol))
        {
            isImprove = true;
        }

        if (shift20_intra_fi(sol))
        {
            isImprove = true;
        }

        if (shift20_inter_fi(sol))
        {
            isImprove = true;
        }

        if (insert_fi(sol))
        {
            isImprove = true;
        }

        if (remove_fi(sol))
        {
            isImprove = true;
        }


        //sol->print_solution();
        //cout << "------------------------------------";
    } while (isImprove);
    return isImprove;

    //bool isImproved = true;
    //int ls_loop = 0;
    //int eta_nonimp = 10;
    //while (ls_loop < eta_nonimp)
    //{
    //	ls_loop++;
    //	isImproved = false;

    //	double origin = 0.0;
    //	bool origin_feasible = false;

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (swap_intra_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		cout << "swap_intra_fi is wrong";
    //		system("pause");
    //	}

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (swap_inter_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		cout << "swap_inter_fi is wrong";
    //		system("pause");
    //	}

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (relocate_intra_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		cout << "relocate_intra_fi is wrong";
    //		system("pause");
    //	}

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (relocate_inter_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		sol->print_solution();
    //		sol->print_xvalues();
    //		sol->print_yvalues();
    //		cout << "relocate_inter_fi is wrong";
    //		system("pause");
    //	}

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (remove_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		cout << "remove_fi is wrong";
    //		system("pause");
    //	}

    //	origin = sol->objfunc;
    //	origin_feasible = sol->isFeasible;
    //	if (insert_fi(sol))
    //	{
    //		isImproved = true;
    //	}
    //	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
    //	{
    //		cout << "insert_fi is wrong";
    //		system("pause");
    //	}

    //	if (!isImproved)
    //	{
    //		break;
    //	}

    //	//cout << "Check the improve by Local_Search obj:" << sol->objfunc << endl;
    //	return isImproved;
    //}
}

//bool isImproved = true;
//int ls_loop = 0;
//int eta_nonimp = 10;
//while (ls_loop < eta_nonimp)
//{
//	ls_loop++;
//	isImproved = false;

//	double origin = 0.0;
//	bool origin_feasible = false;

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (swap_intra_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		cout << "swap_intra_fi is wrong";
//		system("pause");
//	}

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (swap_inter_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		cout << "swap_inter_fi is wrong";
//		system("pause");
//	}

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (relocate_intra_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		cout << "relocate_intra_fi is wrong";
//		system("pause");
//	}

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (relocate_inter_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		sol->print_solution();
//		sol->print_xvalues();
//		sol->print_yvalues();
//		cout << "relocate_inter_fi is wrong";
//		system("pause");
//	}

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (remove_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		cout << "remove_fi is wrong";
//		system("pause");
//	}

//	origin = sol->objfunc;
//	origin_feasible = sol->isFeasible;
//	if (insert_fi(sol))
//	{
//		isImproved = true;
//	}
//	if (sol->objfunc > origin + 0.000001 and !sol->isFeasible)
//	{
//		cout << "insert_fi is wrong";
//		system("pause");
//	}

//	if (!isImproved)
//	{
//		break;
//	}

//	//cout << "Check the improve by Local_Search obj:" << sol->objfunc << endl;
//	return isImproved;
//}
//}
