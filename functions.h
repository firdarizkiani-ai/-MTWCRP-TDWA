#pragma once
#include <iostream>
#include <cmath>
#include <algorithm>
#include <ctime>

using namespace std;

void swap(double& a, double& b);
void find_and_remove_element(vector<int>& set, int node);
int partition(vector<vector<double>>& arr, int low, int high, bool order);
void quickSort(vector<vector<double>>& arr, int low, int high, bool order);
void remove_position_element(vector<int>& set, int pos);
