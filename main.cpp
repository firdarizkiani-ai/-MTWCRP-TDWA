#include <iostream>
#include <vector>
#include "class.h"
#include <algorithm>

using namespace std;

vector<string> instances = 
{
  //"c101_5.txt"
  //"c201_5.txt"  
  //"r101_5.txt"
  //"r201_5.txt"
  //"rc101_5.txt"
  "rc201_5.txt"
  
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
            for (int time = 0; time < numtry; time++)
            {
                /* Implementing the proposed algorithm */
                int eta_shake = 10;       // the number of times that shaking operators are applied
                string shake_strategy = "random";
                int eta_max = 1000;        // Number of maximum iteration
                int eta_nonimp = 100;      // Number of maximum non_improve

                double START, END;
                START = clock();

                MasterProblem* master = new MasterProblem(*data);
                VNS* algorithm = new VNS(*data, *master);
                algorithm->implement(eta_max, eta_nonimp, shake_strategy, eta_shake);

                END = clock();

                if (algorithm->bSol->isFeasible)
                {
                    cout << "numtry :" << time << endl;
                    cout << "The best solution: " << algorithm->bSol->objfunc << "\t" << endl;
                    cout << "VNS-math's processing time: " << (END - START) / CLOCKS_PER_SEC << "\t" << endl;
                }
                else
                {
                    cout << "Nofeasible" << "\t" << (END - START) / CLOCKS_PER_SEC << "\t" << endl;
                }

                master->freeMaster();
                delete master;
                algorithm->freeVNS();
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
