#include <iostream>
#include <vector>
#include "class.h"
#include <algorithm>

using namespace std;

vector<string> instances =
{
  "c101_5.txt"
  //"c201_5.txt"  
  //"r101_5.txt"
  //"r201_5.txt"
  //"rc101_5.txt"
  //"rc201_5.txt"

  //"c101_10.txt"
  //"c201_10.txt"  
  //"r101_10.txt"
  //"r201_10.txt"
  //"rc101_10.txt"
  //"rc201_10.txt"

  //"c101_12.txt"
  //"c201_12.txt"  
  //"r101_12.txt"
  //"r201_12.txt"
  //"rc101_12.txt"
  //"rc201_12.txt"

  //"c101_15.txt"
  //"c201_15.txt"  
  //"r101_15.txt"
  //"r201_15.txt"
  //"rc101_15.txt"
  //"rc201_15.txt"

};

int main()
{
    try
    {
        int numtry = 5;

        for (const string& address : instances)
        {
            srand(0);
            DataProb* data = new DataProb();
            data->readData(address);

            // runtimes for each instance: numtry
            cout << address << "\t";
            double runtime = 0.0;
            for (int time = 0; time < numtry; time++)
            {
                /* Implementing the proposed algorithm */
                //int eta_nonimp = 20;		// N non_improve
                //int eta_cool = 5;			// number of iterations for updating temperature and check newbestresult
                //double T0 = 100;			// initial temperature
                //double Tf = 10;				// final temperature
                //double alpha = 0.93;		// cooling rate
                //double beta = 1;			// coefficent in accepting worse solution
                int eta_shake = 10;       // the number of times that shaking operators are applied
                string shake_strategy = "random";
                int eta_max = 1000;        // Number of maximum iteration
                int eta_nonimp = 100;      // Number of maximum non_improve

                double START, END;
                START = clock();

                MasterProblem* master = new MasterProblem(*data);
                SA* algorithm = new SA(*data, *master);
                //algorithm->implement(eta_nonimp, eta_cool, T0, Tf, alpha, beta);
                //algorithm->implement(eta_nonimp, eta_cool, T0, Tf, alpha, beta, shake_strategy, eta_shake);
                algorithm->implement(eta_max, eta_nonimp, shake_strategy, eta_shake);

                END = clock();
                runtime = (END - START) / CLOCKS_PER_SEC;

                if (runtime < 1200 && algorithm->bSol->isFeasible)
                {
                    cout << "numtry :" << time << endl;
                    cout << "The best solution: " << algorithm->bSol->objfunc << "\t" << endl;
                    cout << "SA's processing time: " << (END - START) / CLOCKS_PER_SEC << "\t" << endl;
                }

                if (runtime > 1200 && algorithm->bSol->isFeasible)
                {
                    cout << "numtry :" << time << endl;
                    cout << "Iterations stops due to time limit" << endl;
                    cout << "The best solution: " << algorithm->bSol->objfunc << "\t" << endl;
                    cout << "SA's processing time: " << (END - START) / CLOCKS_PER_SEC << "\t" << endl;
                }
                if (runtime > 1200 && algorithm->bSol->isFeasible_relaxed)
                {
                    cout << "Nofeasible" << "\t" << (END - START) / CLOCKS_PER_SEC << "\t" << endl;
                }

                master->freeMaster();
                delete master;
                algorithm->freeSA();
                delete algorithm;
            }
            cout << endl;
            delete data;
        }
    }
    catch (GRBException& e)
    {
        cerr << "Gurobi error code = " << e.getErrorCode() << "\n" << e.getMessage() << endl;
        return 1;
    }
    catch (exception& e)
    {
        cerr << "Standard exception: " << e.what() << endl;
        return 1;
    }
    return 0;
}
