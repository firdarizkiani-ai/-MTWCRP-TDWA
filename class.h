
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include "functions.h"
#include "gurobi_c++.h"

using namespace std;

// ======== Small helper typedefs for Gurobi ========
typedef vector<vector<GRBVar>>        NumVar2Matrix;
typedef vector<NumVar2Matrix>         NumVar3Matrix;
typedef vector<NumVar3Matrix>         NumVar4Matrix;

typedef vector<bool>                  BoolVector1D;
typedef vector<BoolVector1D>          BoolVector2D;
typedef vector<BoolVector2D>          BoolVector3D;
typedef vector<BoolVector3D>          BoolVector4D;

typedef vector<int>                   IntVector1D;
typedef vector<IntVector1D>           IntVector2D;
typedef vector<IntVector2D>           IntVector3D;
typedef vector<IntVector3D>           IntVector4D;

typedef vector<pair<int, int>>         PairVector1D;
typedef vector<PairVector1D>          PairVector2D;

typedef vector<double>                DoubleVector1D;
typedef vector<DoubleVector1D>        DoubleVector2D;
typedef vector<DoubleVector2D>        DoubleVector3D;
typedef vector<DoubleVector3D>        DoubleVector4D;

#define ERROR 0.0001

// ======================== Data ========================
class DataProb
{
public:
	vector<int>    stringID;
	vector<int>    xcoord;
	vector<int>    ycoord;
	vector<double> demand;        // must be double
	vector<double> demandrate;
	vector<int>    initial;
	vector<vector<double>> dist;

	int    numVertex;
	int    numTruck;
	int    numFreqMax;
	int    numTripMax;
	double tmax;
	int    vehicle_capacity;
	int    TFL;
	int    fixedCost;
	int    BigM;
	int    remain;
	int    L;
	int    U;
	int    upper_limit_waste;
	int    penalty;

	void readData(string address);
	void printData();

	DataProb();
	~DataProb();
};

// ==================== MasterProblem ====================
class MasterProblem
{
public:
	DataProb& data;

	// >>> persistent Gurobi objects (were commented before)
	GRBEnv env;
	GRBModel model;
	GRBLinExpr objExpr;
	vector<GRBConstr> changingConstraints; // holds dynamic constraints

	// Decision variables
	NumVar2Matrix pickq;           // pickq[i][f]
	vector<GRBVar> tau_e;          // tau_e[i]
	NumVar2Matrix ptime;           // ptime[i][f]
	NumVar3Matrix depa;            // depa[i][k][t]
	vector<GRBVar> exceed_waste;   // exceed_waste[i]

	MasterProblem(DataProb& d);
	~MasterProblem();
	void freeMaster();
};

// ======================== Solution ========================
class Solution
{
public:
	MasterProblem& master;
	DataProb& data;

	PairVector2D    allroute;    // routes per truck with sentinels
	IntVector1D     num_visited;
	IntVector1D     used_trip;
	int             usedTruck;

	BoolVector4D    x_values;    // x[i][j][k][t]
	BoolVector4D    y_values;    // y[v][k][f][t]
	DoubleVector1D  remain_values;

	double          objfunc;
	bool            isFeasible;
	bool            isFeasible_relaxed;

	void update_related_info();
	void solve_masterproblem();

	// neighborhood ops as before
	void insert_a_bin(int route_index, int pos_index, pair<int, int> _bin, const bool skip_solve_masterproblem = false);
	void insert_a_bin2(int route_index, int pos_index, pair<int, int> _bin, const bool skip_solve_masterproblem = false);
	void remove_a_bin(int route_index, int pos_index, const bool skip_solve_masterproblem = false);
	void remove_a_bin2(int route_index, int pos_index, const bool skip_solve_masterproblem = false);
	void swap_two_bin(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem = false);
	void swap_two_bin2(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem = false);
	void relocate_a_bin(int route1_index, int pos1_index, int route2_index, int pos2_index, const bool skip_solve_masterproblem = false);

	//void swap_two_bin(int route1_index, int pos1_index, int route2_index, int pos2_index);

	void copy_from_sol(Solution* other);
	void freeSolution();
	void resetSolution();

	/* support functions for testing */
	void print_solution();
	void print_routes();
	void print_xvalues();
	void print_yvalues();

	Solution(DataProb& data, MasterProblem& master);
	~Solution();
};

// ============================ SA ============================
class SA
{
public:
	DataProb& data;
	MasterProblem& master;

	Solution* iSol, * tSol, * cSol, * bSol, * ttSol;
	double CPU;

	/*
	// Shaking operators
	bool two_opt_star_shake(Solution* sol, int k);
	bool random_remove_destroy(Solution* sol, int k);

	// Shaking selection strategies
	int selectShakingOperator(string strategy, int current_k, int kmax);
	int rouletteWheelSelection();
	void updateOperatorScores(int operator_idx, bool success, double improvement);
	bool eliminate_shortest_routes(Solution* sol, int k);
	bool worst_cost_removal(Solution* sol, int k);
	bool shaw_remove_destroy(Solution* sol, int k);
	bool route_merge_split(Solution* sol, int k);
	bool regret_k_insert(Solution* sol, vector<pair<int, int>>& removed_bins, int k_regret);
	*/
	
	int rouletteWheelSelection();
	//int rouletteWheelSelectionRepair();
	void updateOperatorScores(int operator_idx, bool success, double improvement);
	//void updateRepairScores(int repair_idx, bool success, double improvement);
	
	/*
	// DESTROY METHODS
	bool destroy_two_opt_star(Solution* sol, int k, vector<pair<int, int>>& removed_bins);
	bool destroy_random_remove(Solution* sol, int k, vector<pair<int, int>>& removed_bins);
	bool destroy_eliminate_shortest(Solution* sol, int k, vector<pair<int, int>>& removed_bins);
	bool destroy_worst_cost(Solution* sol, int k, vector<pair<int, int>>& removed_bins);
	bool destroy_route_merge_split(Solution* sol, int k, vector<pair<int, int>>& removed_bins);

	// REPAIR METHODS
	bool repair_random(Solution* sol, vector<pair<int, int>>& removed_bins);
	bool repair_greedy(Solution* sol, vector<pair<int, int>>& removed_bins);
	bool repair_regret(Solution* sol, vector<pair<int, int>>& removed_bins);

	// COMBINED
	bool shake_combined(Solution* sol, int k, int destroy_method, int repair_method);
	*/
	
	bool regret_k_insert(Solution* sol, vector<pair<int, int>>& removed_bins, int k_regret);
	bool two_opt_star_shake(Solution* sol, int k);
	bool random_remove_destroy(Solution* sol, int k);
	bool eliminate_shortest_routes(Solution* sol, int k);
	bool worst_cost_removal(Solution* sol, int k);
	bool route_merge_split(Solution* sol, int k);
	//bool shaw_remove_destroy(Solution* sol, int k);
		

	// Operator performance tracking (untuk adaptive)
	vector<double> operator_weights;    // Bobot untuk roulette wheel
	vector<int> operator_success_count; // Jumlah sukses
	vector<int> operator_total_count;   // Total penggunaan
	vector<double> operator_avg_improvement; // Rata-rata improvement

	// Repair operator tracking
	vector<double> repair_weights;
	vector<int> repair_success_count;
	vector<int> repair_total_count;
	vector<double> repair_avg_improvement;

	void createInitialSolution();
	//void implement(int eta_nonimp, int eta_cool, double T0, double Tf, double alpha, double beta);
	//void implement(int eta_nonimp, int eta_cool, double T0, double Tf, double alpha, double beta, string shake_strategy = "random", int eta_shake = 1);
	void implement(int eta_max, int eta_nonimp, string shake_strategy, int eta_shake_max);
	//void perturbation(Solution* sol);
	//void RVND(Solution* sol);
	//void large_neighborhood_perturbation(Solution* sol);

	/* local search with first improvement (fi) strategy */
	//bool swap_intra_fi(Solution* sol);
	//bool swap_inter_fi(Solution* sol);
	//bool relocate_intra_fi(Solution* sol);
	//bool relocate_inter_fi(Solution* sol);

	//bool insert_fi(Solution* sol);				// may help to handle infeasible solutions in terms of excedding UB
	//bool remove_fi(Solution* sol);				// may help to remove unneccessary visited bins
	bool local_search(Solution* sol);// , int max_iterations);
	
	bool swap_intra_random(Solution* sol);
	bool swap_inter_random(Solution* sol);
	bool relocate_intra_random(Solution* sol);
	bool relocate_inter_random(Solution* sol);
	bool remove_random(Solution* sol);
	bool insert_random(Solution* sol);
	bool two_opt_intra(Solution* sol);
	bool three_opt_intra(Solution* sol);
	//bool relocation_heuristic(Solution* sol);	
	bool swap_intra_fi(Solution* sol);
	bool relocate_intra_fi(Solution* sol);
	bool swap_inter_fi(Solution* sol);
	bool relocate_inter_fi(Solution* sol);
	bool remove_fi(Solution* sol);
	bool insert_fi(Solution* sol);

	bool swap_intra_random2(Solution* sol);
	bool swap_inter_random2(Solution* sol);
	bool relocate_intra_random2(Solution* sol);
	bool relocate_inter_random2(Solution* sol);
	bool remove_random2(Solution* sol);
	bool insert_random2(Solution* sol);
	bool two_opt_intra2(Solution* sol);

	bool swap22_intra_random(Solution* sol);
	bool swap22_inter_random(Solution* sol);
	bool shift20_intra_random(Solution* sol);
	bool shift20_inter_random(Solution* sol);

	bool swap22_intra_fi(Solution* sol);
	bool swap22_inter_fi(Solution* sol);
	bool shift20_intra_fi(Solution* sol);
	bool shift20_inter_fi(Solution* sol);

	void freeSA();
	SA(DataProb& data, MasterProblem& master);
	~SA();
};
