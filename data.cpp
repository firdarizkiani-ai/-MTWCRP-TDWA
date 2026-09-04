#include "class.h"

double euclidean_distance(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	return std::sqrt(dx * dx + dy * dy);
}

DataProb::DataProb() {}

DataProb::~DataProb() {}

void DataProb::readData(string address)
{
	ifstream file(address);

	string temp1, temp2, temp3, temp4, temp5, temp6;
	string temp;
	file >> numVertex >> numTruck >> numFreqMax >> tmax >> vehicle_capacity >> TFL >> numTripMax >> fixedCost >> BigM >> remain >> L >> U >> upper_limit_waste >> penalty;
	file >> temp1 >> temp2 >> temp3 >> temp4 >> temp5 >> temp6;
	if (temp1 != "StringID" or temp2 != "x" or temp3 != "y" or temp4 != "demand" or temp5 != "demandrate" or temp6 != "initial")
	{
		cout << "1. Error data format!";
	}
	else
	{

		for (int i = 0; i < numVertex; i++) {
			file >> temp;
			stringID.push_back(stoi(temp));
			file >> temp;
			xcoord.push_back(stoi(temp));
			file >> temp;
			ycoord.push_back(stoi(temp));
			file >> temp;
			demand.push_back(stoi(temp));
			file >> temp;
			demandrate.push_back(stod(temp));
			file >> temp;
			initial.push_back(stoi(temp));
		}
	}

	//calculate the distance between each Vertex
	vector<double> temp_vec;
	for (int i = 0; i < numVertex; i++) {
		temp_vec.clear();
		for (int j = 0; j < numVertex; j++) {
			temp_vec.push_back(euclidean_distance(xcoord[i], ycoord[i], xcoord[j], ycoord[j]));
		}
		dist.push_back(temp_vec);
	}
}

void DataProb::printData()
{
	cout << "numVertex: " << numVertex << endl;
	cout << "numVeh: " << numTruck << endl;
	cout << "maxVisit: " << numFreqMax << endl;
	cout << "Tmax: " << tmax << endl;
	cout << "capVeh: " << vehicle_capacity << endl;
	cout << "TFL: " << TFL << endl;
	cout << "numTrip: " << numTripMax << endl;
	cout << "fixedCost: " << fixedCost << endl;
	cout << "BigM: " << BigM << endl;
	cout << "remain: " << remain << endl;
	cout << "L: " << L << endl;
	cout << "U: " << U << endl;

	cout << "ID\txcoord\tycoord\tdemand\trate\tdemand0\n";
	for (int i = 0; i < numVertex; i++)
	{
		cout << stringID[i] << "\t" << xcoord[i] << "\t" << ycoord[i] << "\t" << demand[i] << "\t" << demandrate[i] << "\t" << initial[i] << "\n";
	}

	cout << "Distance matrix:\n";
	for (int i = 0; i < numVertex; i++)
	{
		for (int j = 0; j < numVertex; j++)
		{
			cout << dist[i][j] << "\t";
		}
		cout << endl;
	}
}
