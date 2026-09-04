// lebih baik hasilnya dan lebih cepat
//VNS (Variable Neighborhood Search) with Multiple Shaking Operators
#include "class.h"
#include <ctime>
#include <cmath>
#include <algorithm>
#include <unordered_map>
using std::cout; using std::endl;

SA::SA(DataProb& d, MasterProblem& m) : data(d), master(m)
{
    iSol = new Solution(data, master);
    tSol = new Solution(data, master);
    cSol = new Solution(data, master);
    bSol = new Solution(data, master);
    CPU = 0.0;

    // Inisialisasi tracking untuk adaptive strategy (3 operators)
    int num_operators = 4; // two_opt_star, random_remove_destroy, eliminate_shortest_routes, worst_cost
    operator_weights.resize(num_operators, 1.0);
    operator_success_count.resize(num_operators, 0);
    operator_total_count.resize(num_operators, 0);
    operator_avg_improvement.resize(num_operators, 0.0);
}

SA::~SA() {}

void SA::freeSA()
{
    iSol->freeSolution();
    tSol->freeSolution();
    cSol->freeSolution();
    bSol->freeSolution();
    delete iSol;
    delete tSol;
    delete cSol;
    delete bSol;
}

// ========== ROULETTE WHEEL SELECTION ==========
int SA::rouletteWheelSelection()
{
    double total_weight = 0.0;
    for (double w : operator_weights)
    {
        total_weight += w;
    }

    if (total_weight <= 0.0)
    {
        return rand() % operator_weights.size();
    }

    double random_val = ((double)rand() / RAND_MAX) * total_weight;

    double accumulated = 0.0;
    for (int i = 0; i < operator_weights.size(); i++)
    {
        accumulated += operator_weights[i];
        if (random_val <= accumulated)
        {
            return i;
        }
    }

    return operator_weights.size() - 1;
}

// ========== UPDATE OPERATOR SCORES ==========
void SA::updateOperatorScores(int operator_idx, bool success, double improvement)
{
    operator_total_count[operator_idx]++;

    if (success)
    {
        operator_success_count[operator_idx]++;

        double n = operator_success_count[operator_idx];
        operator_avg_improvement[operator_idx] =
            ((n - 1) * operator_avg_improvement[operator_idx] + improvement) / n;
    }

    double success_rate = (double)operator_success_count[operator_idx] /
        (double)operator_total_count[operator_idx];

    double epsilon = 0.1;
    operator_weights[operator_idx] = epsilon +
        success_rate * 10.0 +
        operator_avg_improvement[operator_idx] * 0.1;

    double max_weight = *max_element(operator_weights.begin(), operator_weights.end());
    if (max_weight > 100.0)
    {
        for (double& w : operator_weights)
        {
            w /= max_weight;
        }
    }
}

// ========== UTILITY: REGRET-K INSERTION ==========
bool SA::regret_k_insert(Solution* sol, vector<pair<int, int>>& removed_bins, int k_regret)
{
    if (removed_bins.empty()) return true;

    const int MAX_POS_SAMPLE = 200;
    const bool USE_SAMPLING = true;

    while (!removed_bins.empty())
    {
        struct InsertionCand {
            int route = -1;
            int pos = -1;
            double cost = 1e100;
        };
        struct NodeEval {
            int idx;
            vector<InsertionCand> best;
        };

        vector<NodeEval> evals;
        evals.reserve(removed_bins.size());

        for (int b = 0; b < (int)removed_bins.size(); b++)
        {
            NodeEval nev;
            nev.idx = b;
            vector<InsertionCand> pool;

            int sampled = 0;
            for (int r = 0; r < data.numTruck; r++)
            {
                int route_size = sol->allroute[r].size();
                if (route_size < 2) continue;

                for (int pos = 1; pos < route_size; pos++)
                {
                    if (pos < route_size - 1 && sol->allroute[r][pos].first == 0) continue;
                    if (USE_SAMPLING && sampled >= MAX_POS_SAMPLE) break;
                    if (USE_SAMPLING) sampled++;

                    double old_obj = sol->objfunc;
                    sol->insert_a_bin(r, pos, removed_bins[b]);

                    if (sol->isFeasible_relaxed)
                    {
                        double new_obj = sol->objfunc;
                        InsertionCand c;
                        c.route = r;
                        c.pos = pos;
                        c.cost = (new_obj - old_obj);
                        pool.push_back(c);
                    }

                    sol->remove_a_bin(r, pos, true);
                }
                if (USE_SAMPLING && sampled >= MAX_POS_SAMPLE) break;
            }

            sort(pool.begin(), pool.end(), [](const InsertionCand& a, const InsertionCand& b) {
                return a.cost < b.cost;
                });

            int take = std::min(std::max(1, k_regret), (int)pool.size());
            pool.resize(take);
            nev.best = pool;

            if (nev.best.empty())
            {
                return false;
            }

            evals.push_back(nev);
        }

        struct Scored {
            int idx;
            double regret;
            InsertionCand chosen;
        };
        vector<Scored> scored;
        scored.reserve(evals.size());

        for (auto& e : evals)
        {
            double cost1 = e.best[0].cost;
            double cost2 = (e.best.size() >= 2 ? e.best[1].cost : cost1 + 1e6);
            double regret = cost2 - cost1;

            Scored s;
            s.idx = e.idx;
            s.regret = regret;
            s.chosen = e.best[0];
            scored.push_back(s);
        }

        sort(scored.begin(), scored.end(), [](const Scored& a, const Scored& b) {
            return a.regret > b.regret;
            });

        Scored sel = scored[0];
        int r = sel.chosen.route;
        int p = sel.chosen.pos;

        if (r < 0 || r >= data.numTruck) return false;
        if (p < 1 || p >(int)sol->allroute[r].size())
        {
            p = std::min(p, (int)sol->allroute[r].size());
            if (p < 1) p = 1;
        }

        pair<int, int> bin_to_insert = removed_bins[sel.idx];
        sol->insert_a_bin(r, p, bin_to_insert);

        if (!sol->isFeasible_relaxed)
        {
            sol->remove_a_bin(r, p, true);

            bool inserted = false;
            for (int tries = 0; tries < 20 && !inserted; tries++)
            {
                int rr = rand() % data.numTruck;
                int rs = sol->allroute[rr].size();
                if (rs < 2) continue;

                int pp = 1 + rand() % (rs - 1);
                if (pp < 1 || pp >= rs) continue;
                if (pp < rs - 1 && sol->allroute[rr][pp].first == 0) continue;

                sol->insert_a_bin(rr, pp, bin_to_insert);
                if (sol->isFeasible_relaxed) inserted = true;
                else sol->remove_a_bin(rr, pp, true);
            }
            if (!inserted) return false;
        }

        removed_bins.erase(removed_bins.begin() + sel.idx);
    }

    sol->solve_masterproblem();
    return sol->isFeasible_relaxed;
}

// ========== SHAKING OPERATOR 1: TWO_OPT_STAR ==========
bool SA::two_opt_star_shake(Solution* sol, int k)
{
    bool isChanged = false;
    int max_attempts = k * 3;
    int attempt_count = 0;

    while (attempt_count < max_attempts)
    {
        attempt_count++;

        vector<int> valid_routes;
        for (int r = 0; r < data.numTruck; r++)
        {
            int cust_count = 0;
            for (auto& bin : sol->allroute[r])
            {
                if (bin.first != 0) cust_count++;
            }
            if (cust_count >= 1)
            {
                valid_routes.push_back(r);
            }
        }

        if (valid_routes.size() < 2)
        {
            return isChanged;
        }

        int r1 = valid_routes[rand() % valid_routes.size()];
        int r2 = valid_routes[rand() % valid_routes.size()];

        int retry_count = 0;
        while (r2 == r1 && valid_routes.size() > 1 && retry_count < 10)
        {
            r2 = valid_routes[rand() % valid_routes.size()];
            retry_count++;
        }

        if (r1 == r2)
        {
            continue;
        }

        Solution* backup = new Solution(data, master);
        backup->copy_from_sol(sol);

        vector<int> cust_pos_r1, cust_pos_r2;
        for (int i = 0; i < sol->allroute[r1].size(); i++)
        {
            if (sol->allroute[r1][i].first != 0)
            {
                cust_pos_r1.push_back(i);
            }
        }
        for (int i = 0; i < sol->allroute[r2].size(); i++)
        {
            if (sol->allroute[r2][i].first != 0)
            {
                cust_pos_r2.push_back(i);
            }
        }

        if (cust_pos_r1.empty() || cust_pos_r2.empty())
        {
            delete backup;
            continue;
        }

        int pos1 = cust_pos_r1[rand() % cust_pos_r1.size()];
        int pos2 = cust_pos_r2[rand() % cust_pos_r2.size()];

       sol->swap_two_bin(r1, pos1, r2, pos2, true);

       /*
       sol->solve_masterproblem();

        if (sol->isFeasible_relaxed)
        {
            isChanged = true;
            delete backup;
            break;
        }
        else
        {
            sol->copy_from_sol(backup);
            delete backup;
        }
        */
       
    }

    return isChanged;
}

// ========== SHAKING OPERATOR 2: RANDOM_REMOVE_DESTROY ==========
bool SA::random_remove_destroy(Solution* sol, int k)
{
    bool isChanged = false;

    int total_customers = 0;
    for (int r = 0; r < data.numTruck; r++)
    {
        for (auto& bin : sol->allroute[r])
        {
            if (bin.first != 0)
            {
                total_customers++;
            }
        }
    }

    if (total_customers < 1)
    {
        return false;
    }

    int eta_rem = std::max(1, (int)(0.1 * total_customers));
    int num_to_remove = 1 + rand() % eta_rem;
    num_to_remove = std::min(num_to_remove * k, total_customers);

    Solution* backup = new Solution(data, master);
    backup->copy_from_sol(sol);

    vector<tuple<int, int, pair<int, int>>> all_customers;
    for (int r = 0; r < data.numTruck; r++)
    {
        for (int i = 0; i < sol->allroute[r].size(); i++)
        {
            if (sol->allroute[r][i].first != 0)
            {
                all_customers.push_back(make_tuple(r, i, sol->allroute[r][i]));
            }
        }
    }

    if (all_customers.size() < num_to_remove)
    {
        delete backup;
        return false;
    }

    vector<pair<int, int>> removed_bins;
    random_shuffle(all_customers.begin(), all_customers.end());

    for (int i = 0; i < num_to_remove && i < all_customers.size(); i++)
    {
        removed_bins.push_back(get<2>(all_customers[i]));
    }

    vector<tuple<int, int, pair<int, int>>> selected_to_remove;
    for (int i = 0; i < num_to_remove && i < all_customers.size(); i++)
    {
        selected_to_remove.push_back(all_customers[i]);
    }

    sort(selected_to_remove.begin(), selected_to_remove.end(),
        [](const auto& a, const auto& b) {
            if (get<0>(a) != get<0>(b)) return get<0>(a) > get<0>(b);
            return get<1>(a) > get<1>(b);
        });

    for (auto& item : selected_to_remove)
    {
        int route_idx = get<0>(item);
        int pos_idx = get<1>(item);
        sol->remove_a_bin(route_idx, pos_idx, true);
    }

    random_shuffle(removed_bins.begin(), removed_bins.end());

    bool all_reinserted = true;
    for (auto& bin_pair : removed_bins)
    {
        bool inserted = false;
        int max_tries = 20;
        int tries = 0;

        while (!inserted && tries < max_tries)
        {
            tries++;
            int r = rand() % data.numTruck;
            int route_size = sol->allroute[r].size();
            if (route_size < 2) continue;

            int insert_pos = 1 + rand() % (route_size - 1);

            sol->insert_a_bin(r, insert_pos, bin_pair);

            if (sol->isFeasible_relaxed)
            {
                inserted = true;
            }
            else
            {
                sol->remove_a_bin(r, insert_pos, true);
            }
        }

        if (!inserted)
        {
            all_reinserted = false;
            break;
        }
    }
        
    /*
    sol->solve_masterproblem();

    if (sol->isFeasible_relaxed && all_reinserted)
    {
        isChanged = true;
        delete backup;
    }
    else
    {
        sol->copy_from_sol(backup);
        delete backup;
    }
    */

    return isChanged;
}

// ========== SHAKING OPERATOR 3: ELIMINATE_SHORTEST_ROUTES ==========
bool SA::eliminate_shortest_routes(Solution* sol, int k)
{
    bool isChanged = false;

    vector<pair<int, int>> route_customer_counts;

    for (int r = 0; r < data.numTruck; r++)
    {
        int customer_count = 0;
        for (auto& bin : sol->allroute[r])
        {
            if (bin.first != 0)
            {
                customer_count++;
            }
        }

        if (customer_count > 0)
        {
            route_customer_counts.push_back(make_pair(r, customer_count));
        }
    }

    if (route_customer_counts.size() <= 1)
    {
        return false;
    }

    sort(route_customer_counts.begin(), route_customer_counts.end(),
        [](const pair<int, int>& a, const pair<int, int>& b) {
            return a.second < b.second;
        });

    int num_routes_to_eliminate = min(k, (int)route_customer_counts.size() - 1);
    num_routes_to_eliminate = max(1, num_routes_to_eliminate);

    Solution* backup = new Solution(data, master);
    backup->copy_from_sol(sol);

    vector<pair<int, int>> customers_to_reinsert;
    vector<int> routes_to_eliminate;

    for (int i = 0; i < num_routes_to_eliminate; i++)
    {
        int route_idx = route_customer_counts[i].first;
        routes_to_eliminate.push_back(route_idx);

        for (auto& bin : sol->allroute[route_idx])
        {
            if (bin.first != 0)
            {
                customers_to_reinsert.push_back(bin);
            }
        }
    }

    sort(routes_to_eliminate.begin(), routes_to_eliminate.end(), greater<int>());

    for (int route_idx : routes_to_eliminate)
    {
        for (int pos = sol->allroute[route_idx].size() - 1; pos >= 0; pos--)
        {
            if (sol->allroute[route_idx][pos].first != 0)
            {
                sol->remove_a_bin(route_idx, pos, true);
            }
        }
    }

    random_shuffle(customers_to_reinsert.begin(), customers_to_reinsert.end());

    bool all_reinserted = true;
    for (auto& bin_pair : customers_to_reinsert)
    {
        bool inserted = false;
        int max_attempts = 30;
        int attempt = 0;

        while (!inserted && attempt < max_attempts)
        {
            attempt++;
            int r = rand() % data.numTruck;

            bool is_eliminated = false;
            for (int elim_r : routes_to_eliminate)
            {
                if (r == elim_r)
                {
                    is_eliminated = true;
                    break;
                }
            }
            if (is_eliminated) continue;

            int route_size = sol->allroute[r].size();
            if (route_size < 2) continue;

            int insert_pos = 1 + rand() % (route_size - 1);

            sol->insert_a_bin(r, insert_pos, bin_pair);

            if (sol->isFeasible_relaxed)
            {
                inserted = true;
            }
            else
            {
                sol->remove_a_bin(r, insert_pos, true);
            }
        }

        if (!inserted)
        {
            all_reinserted = false;
            break;
        }
    }

    sol->solve_masterproblem();

    if (sol->isFeasible_relaxed && all_reinserted)
    {
        isChanged = true;
        delete backup;
    }
    else
    {
        sol->copy_from_sol(backup);
        delete backup;
    }
    
    return isChanged;
}

// ========== SHAKING OPERATOR 4: WORST_COST_REMOVAL (FIXED) ==========
bool SA::worst_cost_removal(Solution* sol, int k)
{
    bool isChanged = false;

    // Step 1: Hitung total customers
    int total_customers = 0;
    for (int r = 0; r < data.numTruck; r++)
    {
        for (auto& bin : sol->allroute[r])
        {
            if (bin.first != 0)
            {
                total_customers++;
            }
        }
    }

    if (total_customers < 1)
    {
        return false;
    }

    // Step 2: Tentukan jumlah node yang akan dihapus
    int eta_rem = std::max(1, (int)(0.1 * total_customers));
    int num_to_remove = 1 + rand() % eta_rem;
    num_to_remove = std::min(num_to_remove * k, total_customers);
    num_to_remove = std::min(num_to_remove, (int)(0.3 * total_customers));

    // Step 3: Backup solution
    Solution* backup = new Solution(data, master);
    backup->copy_from_sol(sol);

    // Step 4: Hitung cost contribution untuk setiap customer
    struct CustomerCost {
        int route_idx;
        int pos_idx;
        pair<int, int> bin_freq;
        double cost_contribution;
    };

    vector<CustomerCost> customer_costs;

    for (int r = 0; r < data.numTruck; r++)
    {
        int route_size = sol->allroute[r].size();

        // VALIDASI: Route harus punya minimal 3 elemen (depot-customer-depot)
        if (route_size < 3) continue;

        for (int i = 1; i < route_size - 1; i++)
        {
            // Skip depot
            if (sol->allroute[r][i].first == 0) continue;

            // VALIDASI: Pastikan prev dan next index valid
            if (i - 1 < 0 || i + 1 >= route_size) continue;

            int prev_bin = sol->allroute[r][i - 1].first;
            int curr_bin = sol->allroute[r][i].first;
            int next_bin = sol->allroute[r][i + 1].first;

            // VALIDASI: Pastikan bins valid dalam range
            if (prev_bin < 0 || prev_bin >= data.numVertex) continue;
            if (curr_bin < 0 || curr_bin >= data.numVertex) continue;
            if (next_bin < 0 || next_bin >= data.numVertex) continue;

            // Calculate cost contribution
            double cost_prev_curr = data.dist[prev_bin][curr_bin];
            double cost_curr_next = data.dist[curr_bin][next_bin];
            double cost_prev_next = data.dist[prev_bin][next_bin];

            double cost_contribution = cost_prev_curr + cost_curr_next - cost_prev_next;

            CustomerCost cc;
            cc.route_idx = r;
            cc.pos_idx = i;
            cc.bin_freq = sol->allroute[r][i];
            cc.cost_contribution = cost_contribution;

            customer_costs.push_back(cc);
        }
    }

    if (customer_costs.empty())
    {
        delete backup;
        return false;
    }

    // Step 5: Sort by cost contribution (DESCENDING - worst first)
    sort(customer_costs.begin(), customer_costs.end(),
        [](const CustomerCost& a, const CustomerCost& b) {
            return a.cost_contribution > b.cost_contribution;
        });

    // Step 6: Select top customers to remove
    vector<pair<int, int>> removed_bins;
    vector<CustomerCost> to_remove;

    int actual_remove = std::min(num_to_remove, (int)customer_costs.size());
    for (int i = 0; i < actual_remove; i++)
    {
        to_remove.push_back(customer_costs[i]);
        removed_bins.push_back(customer_costs[i].bin_freq);
    }

    // Step 7: Remove customers (from back to front)
    sort(to_remove.begin(), to_remove.end(),
        [](const CustomerCost& a, const CustomerCost& b) {
            if (a.route_idx != b.route_idx)
                return a.route_idx > b.route_idx;
            return a.pos_idx > b.pos_idx;
        });

    for (auto& cc : to_remove)
    {
        // VALIDASI: Pastikan route dan position masih valid sebelum remove
        if (cc.route_idx >= 0 && cc.route_idx < data.numTruck)
        {
            if (cc.pos_idx >= 0 && cc.pos_idx < sol->allroute[cc.route_idx].size())
            {
                sol->remove_a_bin(cc.route_idx, cc.pos_idx, true);
            }
        }
    }

    // Step 8: Reinsert dengan FIRST FEASIBLE (safe approach)
    random_shuffle(removed_bins.begin(), removed_bins.end());

    bool all_reinserted = true;

    for (auto& bin_pair : removed_bins)
    {
        bool inserted = false;
        int max_tries = 30;
        int tries = 0;

        while (!inserted && tries < max_tries)
        {
            tries++;

            // Pilih route random
            int r = rand() % data.numTruck;

            // VALIDASI: Cek route size
            int route_size = sol->allroute[r].size();
            if (route_size < 2)
            {
                continue; // Route terlalu kecil, skip
            }

            // Pilih insert position (antara 1 dan route_size-1)
            // Hindari insert di posisi 0 (depot awal) dan posisi >= route_size
            int insert_pos = 1 + rand() % (route_size - 1);

            // VALIDASI EXTRA: Pastikan insert_pos dalam range valid
            if (insert_pos < 1 || insert_pos >= route_size)
            {
                continue;
            }

            // VALIDASI: Cek apakah posisi ini aman
            // Jangan insert tepat di depot kecuali depot terakhir
            if (insert_pos < route_size - 1 && sol->allroute[r][insert_pos].first == 0)
            {
                continue; // Skip posisi depot intermediate
            }

            // Try insert
            sol->insert_a_bin(r, insert_pos, bin_pair);

            // Check feasibility
            if (sol->isFeasible_relaxed)
            {
                inserted = true;
            }
            else
            {
                // Remove kembali jika tidak feasible
                // VALIDASI: Pastikan posisi masih valid untuk remove
                if (insert_pos < sol->allroute[r].size())
                {
                    sol->remove_a_bin(r, insert_pos, true);
                }
            }
        }

        if (!inserted)
        {
            all_reinserted = false;
            break;
        }
    }

    // Step 9: Solve master problem
    sol->solve_masterproblem();

    // Step 10: Accept jika feasible dan semua berhasil reinserted
    if (sol->isFeasible_relaxed && all_reinserted)
    {
        isChanged = true;
        delete backup;
    }
    else
    {
        // Rollback jika gagal
        sol->copy_from_sol(backup);
        delete backup;
    }
    
    return isChanged;
}

// ========== SHAKING OPERATOR 5: ROUTE_MERGE_SPLIT ==========
bool SA::route_merge_split(Solution* sol, int k)
{
    bool isChanged = false;

    vector<int> valid_routes;
    for (int r = 0; r < data.numTruck; r++)
    {
        int cust_count = 0;
        for (auto& bin : sol->allroute[r])
        {
            if (bin.first != 0) cust_count++;
        }
        if (cust_count >= 1) valid_routes.push_back(r);
    }

    if (valid_routes.size() < 2) return false;

    int r1 = valid_routes[rand() % valid_routes.size()];
    int r2 = valid_routes[rand() % valid_routes.size()];

    int retry = 0;
    while (r1 == r2 && valid_routes.size() > 1 && retry < 10)
    {
        r2 = valid_routes[rand() % valid_routes.size()];
        retry++;
    }

    if (r1 == r2) return false;

    Solution* backup = new Solution(data, master);
    backup->copy_from_sol(sol);

    vector<pair<int, int>> merged_customers;

    for (auto& bin : sol->allroute[r1])
    {
        if (bin.first != 0) merged_customers.push_back(bin);
    }

    for (auto& bin : sol->allroute[r2])
    {
        if (bin.first != 0) merged_customers.push_back(bin);
    }

    if (merged_customers.empty())
    {
        delete backup;
        return false;
    }

    for (int pos = sol->allroute[r1].size() - 1; pos >= 0; pos--)
    {
        if (sol->allroute[r1][pos].first != 0)
        {
            sol->remove_a_bin(r1, pos, true);
        }
    }

    for (int pos = sol->allroute[r2].size() - 1; pos >= 0; pos--)
    {
        if (sol->allroute[r2][pos].first != 0)
        {
            sol->remove_a_bin(r2, pos, true);
        }
    }

    random_shuffle(merged_customers.begin(), merged_customers.end());

    vector<pair<int, int>> route1_customers;
    vector<pair<int, int>> route2_customers;

    double total_demand = 0.0;
    for (auto& bin : merged_customers)
    {
        total_demand += (bin.first < data.numVertex) ? data.demand[bin.first] : 0.0;
    }

    double target_demand = total_demand / 2.0;
    double current_demand_r1 = 0.0;

    for (auto& bin : merged_customers)
    {
        double bin_demand = (bin.first < data.numVertex) ? data.demand[bin.first] : 0.0;

        if (current_demand_r1 + bin_demand <= target_demand * 1.2)
        {
            route1_customers.push_back(bin);
            current_demand_r1 += bin_demand;
        }
        else
        {
            route2_customers.push_back(bin);
        }
    }

    if (route1_customers.empty() || route2_customers.empty())
    {
        int mid = merged_customers.size() / 2;
        route1_customers.clear();
        route2_customers.clear();

        for (int i = 0; i < mid; i++) route1_customers.push_back(merged_customers[i]);
        for (int i = mid; i < merged_customers.size(); i++) route2_customers.push_back(merged_customers[i]);
    }

    bool all_inserted = regret_k_insert(sol, route1_customers, 2);

    if (all_inserted)
    {
        all_inserted = regret_k_insert(sol, route2_customers, 2);
    }

    sol->solve_masterproblem();

    if (sol->isFeasible_relaxed && all_inserted)
    {
        isChanged = true;
        delete backup;
    }
    else
    {
        sol->copy_from_sol(backup);
        delete backup;
    }

    return isChanged;
}

// ========== VNS MAIN ALGORITHM (Algorithm 1) ==========
void SA::implement(int eta_max, int eta_nonimp, string shake_strategy, int eta_shake_max)
{
    double START, END;
    START = clock();

    // Line 1: σ ← initializeSol()
    createInitialSolution();
    iSol->solve_masterproblem();

    // Line 2: σ', σ* ← σ, σ*_f ← ∅
    cSol->copy_from_sol(iSol);  // σ' (current solution)
    if (iSol->isFeasible)
    {
        bSol->copy_from_sol(iSol); // σ* (best overall)
    }
    tSol->copy_from_sol(iSol);  // σ'' (temporary after shake)

    Solution* bestFeasible = new Solution(data, master); // σ*_f

    // Line 3-5: Check initial feasibility
    if (iSol->isFeasible)
    {
        bestFeasible->copy_from_sol(iSol);
        cout << "Initial solution is FEASIBLE: " << bestFeasible->objfunc << endl;
    }
    else if (iSol->isFeasible_relaxed)
    {
        cout << "Initial solution is FLEXIBLE FEASIBLE: " << bestFeasible->objfunc << endl;
    }
    else
    {
        bestFeasible->objfunc = INFINITY; // No feasible solution yet
        cout << "Initial solution is INFEASIBLE" << endl;
    }

    // Line 6: ι ← 0, k ← 1, ηshake ← 1, R ← ∅
    int iota = 0;           // ι: iteration counter
    int k = 1;              // k: neighborhood index
    int eta_shake = 1;      // ηshake: shaking intensity
    int iota_imp = 0;       // ιimp: last improvement iteration

    cout << "\n=== VNS Configuration ===" << endl;
    cout << "Shaking strategy: " << shake_strategy << endl;
    cout << "Max iterations (ηmax): " << eta_max << endl;
    cout << "Non-improvement limit (ηimp): " << eta_nonimp << endl;
    cout << "Max shaking intensity (ηshake_max): " << eta_shake_max << endl;
    cout << "===========================" << endl;

    const char* operator_names[] = {
    "two_opt_star",
    "random_remove_destroy",
    "eliminate_shortest_routes",
    "worst_cost_removal"  // NEW
    //"shaw_remove_destroy"  // NEW
    //"route_merge_split"  // NEW
    };

    int kmax = 5; // Maximum neighborhood index

    // Line 7: Main VNS loop
    while (iota < eta_max && (iota - iota_imp) < eta_nonimp)
    {
        iota++;
        /*
        // Line 8-12: Select starting point
        if (cSol->objfunc < bestFeasible->objfunc || bestFeasible->objfunc == INFINITY)
        {
            // Line 9: σ'' ← σ' (start from current)
            tSol->copy_from_sol(cSol);
        }
        else
        {
            // Line 11: σ'' ← σ*_f (start from best feasible)
            tSol->copy_from_sol(bestFeasible);
        }
        */

        double obj_before_shake = tSol->objfunc;

        // Line 13: σ'' ← shakeProcedure(σ'', S, ζshake, ηshake, k)
        bool shakeSuccess = false;

        // Di dalam main VNS loop:

        if (shake_strategy == "sequential")
        {
            cout << "Iter " << iota << " | SEQUENTIAL (k=" << k << ")" << endl;

            bool op1 = two_opt_star_shake(tSol, k);
            bool op2 = random_remove_destroy(tSol, k);
            bool op3 = eliminate_shortest_routes(tSol, k);
            bool op4 = worst_cost_removal(tSol, k);  // NEW
            //bool op5 = shaw_remove_destroy(tSol, k);  // NEW
            //bool op5 = route_merge_split(tSol, k);  // NEW

            cout << "  - two_opt_star: " << (op1 ? "OK" : "FAIL") << endl;
            cout << "  - random_remove_destroy: " << (op2 ? "OK" : "FAIL") << endl;
            cout << "  - eliminate_shortest_routes: " << (op3 ? "OK" : "FAIL") << endl;
            cout << "  - worst_cost_removal: " << (op4 ? "OK" : "FAIL") << endl;
            //cout << "  - shaw_remove_destroy: " << (op5 ? "OK" : "FAIL") << endl;
            //cout << "  - route_merge_split: " << (op5 ? "OK" : "FAIL") << endl;

            shakeSuccess = op1 || op2 || op3 || op4;// || op5;
        }
        else if (shake_strategy == "random")
        {
            for (int s = 0; s < eta_shake; s++)
            {
                int selected_op = rand() % 4;  // 0-4

                bool current_success = false;
                if (selected_op == 0)
                    current_success = two_opt_star_shake(tSol, k);
                else if (selected_op == 1)
                    current_success = random_remove_destroy(tSol, k);
                else if (selected_op == 2)
                    current_success = eliminate_shortest_routes(tSol, k);
                else if (selected_op == 3)
                    current_success = worst_cost_removal(tSol, k);  // NEW
                /*
                else if (selected_op == 4)
                    current_success = shaw_remove_destroy(tSol, k);  // NEW
                else if (selected_op == 4)
                    current_success = route_merge_split(tSol, k);  // NEW
                */

                if (current_success) shakeSuccess = true;

                cout << "Iter " << iota << " | RANDOM | " << operator_names[selected_op]
                    << " (k=" << k << "): " << (current_success ? "OK" : "FAIL") << endl;
            }
        }
        else if (shake_strategy == "adaptive")
        {
            for (int s = 0; s < eta_shake; s++)
            {
                int selected_op = rouletteWheelSelection();

                bool current_success = false;
                if (selected_op == 0)
                    current_success = two_opt_star_shake(tSol, k);
                else if (selected_op == 1)
                    current_success = random_remove_destroy(tSol, k);
                else if (selected_op == 2)
                    current_success = eliminate_shortest_routes(tSol, k);
                else if (selected_op == 3)
                    current_success = worst_cost_removal(tSol, k);  // NEW
                /*
                else if (selected_op == 4)
                    current_success = shaw_remove_destroy(tSol, k);  // NEW               
                else if (selected_op == 4)
                    current_success = route_merge_split(tSol, k);  // NEW
                */                

                double improvement = obj_before_shake - tSol->objfunc;
                updateOperatorScores(selected_op, current_success, std::max(0.0, improvement));

                if (current_success) shakeSuccess = true;

                cout << "Iter " << iota << " | ADAPTIVE | " << operator_names[selected_op]
                    << " (k=" << k << "): " << (current_success ? "OK" : "FAIL")
                    << " | W=[" << operator_weights[0] << "," << operator_weights[1]
                    << "," << operator_weights[2] << "," << operator_weights[3] << endl;
                    //"]" << operator_weights[4] << "]" << endl;
            }
        }

        bool isChanged = false;
        int nbMoves = 6;
        int selected_operator = rand() % nbMoves;
        int trial = 15;
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
        } while (!isChanged and count < trial);

        // Line 14: σ'' ← local_search(σ'', N, ζvnd, ζls, ι)

        local_search(tSol);// , 5);
        
        /*
        if (iota % 5 == 0)
        {
            local_search(tSol,3);
        }
        */

        // Line 15: σ'' ← solve_masterproblem(σ'')
        //tSol->solve_masterproblem();

        // Line 16-21: Acceptance criterion
        if (tSol->objfunc < cSol->objfunc - ERROR)
        {
            // Line 17: σ' ← σ'' (accept new solution)
            cSol->copy_from_sol(tSol);

            // Line 18: k ← 1, ηshake ← 1, ιimp ← ι
            k = 1;
            eta_shake = 1;
            iota_imp = iota;

            cout << "  -> ACCEPTED (improvement: " << (cSol->objfunc - tSol->objfunc) << ")" << endl;
        }
        else
        {
            // Line 20: k ← k + 1, ηshake ← max{ηshake + 1, ηshake_max}
            k++;
            if (k > kmax) k = 1;

            eta_shake = std::min(eta_shake + 1, eta_shake_max);

            cout << "  -> REJECTED (no improvement)" << endl;
        }

        // Line 22-24: Update best feasible solution
        if (tSol->isFeasible && tSol->objfunc < bestFeasible->objfunc - ERROR)
        {
            bestFeasible->copy_from_sol(tSol);
            cout << "  *** NEW BEST FEASIBLE: " << bestFeasible->objfunc << " ***" << endl;
        }

        // Periodic progress report
        if (iota % 20 == 0)
        {
            END = clock();
            double runtime = (END - START) / CLOCKS_PER_SEC;
            cout << "\n--- Progress Report ---" << endl;
            cout << "Iteration: " << iota << "/" << eta_max << endl;
            cout << "Non-improvement: " << (iota - iota_imp) << "/" << eta_nonimp << endl;
            cout << "Current k: " << k << ", ηshake: " << eta_shake << endl;
            cout << "Best feasible: " << (bestFeasible->objfunc < INFINITY ?
                std::to_string(bestFeasible->objfunc) : "NONE") << endl;
            cout << "Runtime: " << runtime << "s" << endl;
            cout << "-----------------------\n" << endl;
        }
    }

    END = clock();

    // Line 27: return σ*_f
    cout << "\n=== VNS FINAL RESULTS ===" << endl;
    cout << "Total iterations: " << iota << endl;
    cout << "Last improvement at iteration: " << iota_imp << endl;
    cout << "Processing time: " << (END - START) / CLOCKS_PER_SEC << " seconds" << endl;

    if (bestFeasible->objfunc < INFINITY)
    {
        cout << "Best feasible solution: " << bestFeasible->objfunc << endl;
        bestFeasible->print_solution();

        // Copy to bSol for compatibility
        bSol->copy_from_sol(bestFeasible);
    }
    else
    {
        cout << "No feasible solution found!" << endl;
    }

    if (shake_strategy == "adaptive")
    {
        cout << "\n=== Operator Statistics ===" << endl;
        cout << "Two-opt*: " << operator_success_count[0] << "/" << operator_total_count[0]
            << " (avg imp: " << operator_avg_improvement[0] << ")" << endl;
        cout << "Random-destroy: " << operator_success_count[1] << "/" << operator_total_count[1]
            << " (avg imp: " << operator_avg_improvement[1] << ")" << endl;
        cout << "Eliminate-shortest: " << operator_success_count[2] << "/" << operator_total_count[2]
            << " (avg imp: " << operator_avg_improvement[2] << ")" << endl;
        cout << "Worst-cost-removal: " << operator_success_count[3] << "/" << operator_total_count[3]
            << " (avg imp: " << operator_avg_improvement[3] << ")" << endl;
        cout << "shaw_remove_destroy: " << operator_success_count[4] << "/" << operator_total_count[4]
            << " (avg imp: " << operator_avg_improvement[4] << ")" << endl;
    }

    delete bestFeasible;
}

// ========== HELPER FUNCTIONS ==========

void SA::createInitialSolution()
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

bool SA::local_search(Solution* sol)//, int max_iterations)
{
    bool isImprove = false;
    //int ls_iter = 0;

    do
    {
        //ls_iter++;
        isImprove = false;

        if (swap22_intra_fi(sol)) isImprove = true;
        if (swap22_inter_fi(sol)) isImprove = true;
        if (shift20_intra_fi(sol)) isImprove = true;
        if (shift20_inter_fi(sol)) isImprove = true;

        if (relocate_inter_fi(sol)) isImprove = true;
        if (swap_intra_fi(sol)) isImprove = true;
        if (insert_fi(sol)) isImprove = true;
        if (remove_fi(sol)) isImprove = true;
        if (relocate_intra_fi(sol)) isImprove = true;
        if (swap_inter_fi(sol)) isImprove = true;       
        if (two_opt_intra(sol)) isImprove = true;
        if (three_opt_intra(sol)) isImprove = true;
        
        
    } while (isImprove);//&& ls_iter < max_iterations); // TAMBAHKAN BATAS ITERASI

    return isImprove;
}
