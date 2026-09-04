#include <vector>
#include <algorithm>
#include "functions.h"

void swap(double& a, double& b) {
    double t = a;
    a = b;
    b = t;
}

int partition(vector<vector<double>>& arr, int low, int high, bool order) {
    double pivot = arr[high][0];
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++) {
        if (order) {
            if (arr[j][0] < pivot) {
                i++;
                swap(arr[i][0], arr[j][0]);
                swap(arr[i][1], arr[j][1]);
            }
        }
        else {
            if (arr[j][0] > pivot) {
                i++;
                swap(arr[i][0], arr[j][0]);
                swap(arr[i][1], arr[j][1]);
            }
        }
    }
    swap(arr[i + 1][0], arr[high][0]);
    swap(arr[i + 1][1], arr[high][1]);
    return (i + 1);
}

void quickSort(vector<vector<double>>& arr, int low, int high, bool order) {
    if (low < high) {
        int pi = partition(arr, low, high, order);
        quickSort(arr, low, pi - 1, order);
        quickSort(arr, pi + 1, high, order);
    }
}

void find_and_remove_element(vector<int>& set, int node) {
    auto it = std::find(set.begin(), set.end(), node);
    if (it != set.end()) {
        set.erase(it);
    }
}

void remove_position_element(vector<int>& set, int pos) {
    set.erase(set.begin() + pos);
}