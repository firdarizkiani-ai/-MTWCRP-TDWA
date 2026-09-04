#include "class.h"

MasterProblem::MasterProblem(DataProb& d)
    : data(d), env(), model(env), objExpr(0.0)
{
    // quiet log
    model.set(GRB_IntParam_OutputFlag, 0);

    // ---------- Decision variables ----------
    // pickq[i][f] for all i in [0..n-1], f in [0..F-1]
    pickq.assign(data.numVertex, vector<GRBVar>(data.numFreqMax));
    for (int i = 0; i < data.numVertex; ++i)
        for (int f = 0; f < data.numFreqMax; ++f)
            pickq[i][f] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "pickq_" + to_string(i) + "_" + to_string(f));

    tau_e.assign(data.numVertex, GRBVar());
    for (int i = 0; i < data.numVertex; ++i)
        tau_e[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "tau_e_" + to_string(i));

    ptime.assign(data.numVertex, vector<GRBVar>(data.numFreqMax));
    for (int i = 0; i < data.numVertex; ++i)
        for (int f = 0; f < data.numFreqMax; ++f)
            ptime[i][f] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "ptime_" + to_string(i) + "_" + to_string(f));

    depa.assign(data.numVertex, NumVar2Matrix());
    depa.resize(data.numVertex);
    for (int i = 0; i < data.numVertex; ++i) {
        depa[i].assign(data.numTruck, vector<GRBVar>(data.numTripMax));
        for (int k = 0; k < data.numTruck; ++k)
            for (int t = 0; t < data.numTripMax; ++t)
                depa[i][k][t] = model.addVar(0.0, data.tmax - data.dist[i][0], 0.0, GRB_CONTINUOUS,
                    "depa_" + to_string(i) + "_" + to_string(k) + "_" + to_string(t));
    }

    exceed_waste.assign(data.numVertex, GRBVar());
    for (int i = 0; i < data.numVertex; ++i)
        exceed_waste[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "exceed_" + to_string(i));

    model.update();

    // ---------- Objective ----------
    objExpr = 0.0;
    for (int i = 1; i < data.numVertex; ++i)
        objExpr += data.penalty * exceed_waste[i];
    model.setObjective(objExpr, GRB_MINIMIZE);

    // ---------- Static constraints (not depending on x/y) ----------
    // ptime[i][0] == 0, for i>=1
    for (int i = 1; i < data.numVertex; ++i)
        model.addConstr(ptime[i][0] == 0.0, "c1_ptime0_" + to_string(i));

    // sum_f pickq[i][f] + tau_e[i] - initial[i] - tmax*demandrate[i] == 0
    for (int i = 1; i < data.numVertex; ++i) {
        GRBLinExpr sumPick = 0.0;
        for (int f = 1; f < data.numFreqMax; ++f) sumPick += pickq[i][f];
        model.addConstr(sumPick + tau_e[i] - data.initial[i] - data.tmax * data.demandrate[i] == 0.0,
            "c2_mass_" + to_string(i));
        // pickq[i][f] - (ptime[i][f] - ptime[i][f-1]) * demandrate[i] == 0 for f>=2
        for (int f = 2; f < data.numFreqMax; ++f)
            model.addConstr(pickq[i][f] - (ptime[i][f] - ptime[i][f - 1]) * data.demandrate[i] == 0.0,
                "c3_link_" + to_string(i) + "_" + to_string(f));
    }

    // exceed_waste[i] >= tau_e[i] - upper_limit_waste
    for (int i = 1; i < data.numVertex; ++i)
        model.addConstr(exceed_waste[i] - (tau_e[i] - data.upper_limit_waste) >= 0.0,
            "c4_exceed_" + to_string(i));

    // storage limits with TFL
    for (int i = 1; i < data.numVertex; ++i) {
        model.addConstr(ptime[i][1] * data.demandrate[i] + data.initial[i] - data.TFL <= 0.0,
            "c5_tfl1_" + to_string(i));
        for (int f = 2; f < data.numFreqMax; ++f)
            model.addConstr((ptime[i][f] - ptime[i][f - 1]) * data.demandrate[i] - data.TFL <= 0.0,
                "c6_tflf_" + to_string(i) + "_" + to_string(f));
    }

    model.update();
}

MasterProblem::~MasterProblem() {}
void MasterProblem::freeMaster() { /* nothing required for Gurobi */ }
