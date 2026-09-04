#include "class.h"
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include "functions.h"

bool SA::swap_intra_fi(Solution* sol)
{
	//cout << "SwapIntra" << endl;
	for (int r_index = 0; r_index < data.numTruck; r_index++)
	{
		int start_point = 1;
		int end_point = -1;

		for (int t_index = 0; t_index < data.numTripMax; t_index++)
		{
			// Check if both start_point and end_point are 0
			if (sol->allroute[r_index][start_point].first == 0)
			{
				start_point++;
				break;
			}
			else
			{
				// Find end of trip for the current route
				end_point = start_point + 1;
				while (sol->allroute[r_index][end_point].first != 0)
				{
					end_point++;
				}
			}

			for (int posi = start_point; posi < end_point - 1; posi++)
			{
				for (int posj = posi + 1; posj < end_point; posj++)
				{
					double pre_obj = sol->objfunc;
					bool pre_feasible = sol->isFeasible;
					double originobj = sol->objfunc;
					// Swap the two positions
					//cout << "route:" << r_index << " check pos i:" << posi << " check pos j:" << posj << endl;
					//sol->print_routes();
					sol->swap_two_bin(r_index, posi, r_index, posj);
					//sol->print_solution();
					//system("pause");

					if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
					{
						return true;
					}
					else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
					{
						return true;
					}
					else
					{
						sol->swap_two_bin(r_index, posj, r_index, posi);
					}

					//if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
					//{
					//	pre_obj = sol->objfunc;
					//	return true;
					//}
					//else
					//{
					//	// Swap back if no improvement
					//	sol->swap_two_bin(r_index, posj, r_index, posi);
					//	if (abs(sol->objfunc - originobj) > 0.00001)
					//	{
					//		cout << "wrong here swapInra!" << endl;
					//		sol->print_solution();
					//		system("pause");
					//	}
					//}
				}
			}
			// Update start_point for the next trip
			start_point = end_point + 1;
		}
	}
	return false;
}

bool SA::swap_inter_fi(Solution* sol)
{
	//cout << "SwapInter" << endl;
	for (int r1_index = 0; r1_index < data.numTruck; r1_index++)
	{
		int start_point_1 = 1;
		int end_point_1 = -1;

		for (int t1_index = 0; t1_index < data.numTripMax; t1_index++)
		{
			end_point_1 = start_point_1 + 1;

			// Find end of trip for route 1
			while (sol->allroute[r1_index][end_point_1].first != 0)
			{
				end_point_1++;
			}

			for (int posi = start_point_1; posi < end_point_1; posi++)
			{
				for (int r2_index = r1_index; r2_index < data.numTruck; r2_index++)
				{
					// To make sure start_point_2's position is correct
					int start_point_2 = 1;
					int end_point_2 = -1;

					for (int t2_index = 0; t2_index < data.numTripMax; t2_index++)
					{
						end_point_2 = start_point_2 + 1;

						// Find end of trip for route 2
						while (sol->allroute[r2_index][end_point_2].first != 0 and end_point_2 < sol->allroute[r2_index].size())
						{
							end_point_2++;
						}

						if (r1_index == r2_index and t1_index >= t2_index)
						{
						}
						else
						{
							for (int posj = start_point_2; posj < end_point_2; posj++)
							{
								// Ensure posj is within bounds
								if (posj >= sol->allroute[r2_index].size() || posi >= sol->allroute[r1_index].size()) break;

								// Skip if either posi or posj is zero
								if (sol->allroute[r1_index][posi].first == 0 || sol->allroute[r2_index][posj].first == 0)
								{
									continue;
								}
								bool already_in_trip = false;
								for (int k = start_point_2; k < end_point_2; k++)
								{
									if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first)
									{
										already_in_trip = true;
										break;
									}
								}

								if (already_in_trip) continue;

								double pre_obj = sol->objfunc;
								bool pre_feasible = sol->isFeasible;
								//cout << "route_1:" << r1_index << " check pos i:" << posi << " route_2:" << r2_index << " check pos j:" << posj << endl;
								//system("pause");
								//sol->print_solution();

								double originobj = sol->objfunc;
								if (sol->allroute[r1_index][posi].first != 0 and sol->allroute[r2_index][posj].first != 0)
								{
									sol->swap_two_bin(r1_index, posi, r2_index, posj);

									if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
									{
										return true;
									}
									else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
									{
										return true;
									}
									else
									{
										sol->swap_two_bin(r2_index, posj, r1_index, posi);
									}

									//if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
									//{
									//	pre_obj = sol->objfunc;
									//	//sol->print_solution();
									//	return true;
									//}
									//else
									//{
									//	sol->swap_two_bin(r2_index, posj, r1_index, posi);
									//	//if (abs(sol->objfunc - originobj) > 0.00001)
									//	//{
									//	//	cout << "wrong here swapInter!" << endl << endl;
									//	//	sol->print_xvalues();
									//	//	sol->print_yvalues();
									//	//	sol->print_solution();
									//	//	system("pause");
									//	//}
									//}
									////sol->print_solution();
									////system("pause");
								}

							}
						}

						// Update start_point_2 for the next trip in route 2
						start_point_2 = end_point_2 + 1;
					}
				}
			}

			// Check if both start_point and end_point are 0
			if (sol->allroute[r1_index][start_point_1].first == 0 && sol->allroute[r1_index][end_point_1].first == 0)
			{
				break;
			}

			// Update start_point_1 for the next trip in route 1
			start_point_1 = end_point_1 + 1;
		}
	}
	return false;
}

bool SA::relocate_intra_fi(Solution* sol)
{
	//cout << "RelocateIntra" << endl;
	for (int r_index = 0; r_index < data.numTruck; r_index++)
	{
		int start_point = 1;
		int end_point = -1;
		for (int t_index = 0; t_index < data.numTripMax; t_index++)
		{
			end_point = start_point + 1;
			while (end_point < sol->allroute[r_index].size() && sol->allroute[r_index][end_point].first != 0)
			{
				end_point++;
			}

			for (int posi = start_point; posi < end_point; posi++)
			{
				if (posi >= sol->allroute[r_index].size() || sol->allroute[r_index][posi].first == 0) break;
				for (int posj = start_point; posj < end_point; posj++)
				{

					double pre_obj = sol->objfunc;
					bool pre_feasible = sol->isFeasible;
					double originobj = sol->objfunc;

					if (posj != posi)
					{
						//cout << "check route:" << r_index << " posi:" << posi << " posj:" << posj << endl;
						//sol->print_routes();
						//

						sol->relocate_a_bin(r_index, posi, r_index, posj);
						//sol->print_routes();
						//system("pause");
						if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
						{
							return true;
						}
						else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
						{
							return true;
						}
						else
						{
							sol->relocate_a_bin(r_index, posj, r_index, posi);
						}
					}


					//if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
					//{
					//	pre_obj = sol->objfunc;
					//	//sol->print_solution();
					//	return true;
					//}
					//else
					//{
					//	sol->relocate_a_bin(r_index, posj, r_index, posi);
					//	if (abs(sol->objfunc - originobj) > 0.00001)
					//	{
					//		//sol->print_xvalues();
					//		//sol->print_yvalues();
					//		cout << "wrong here smaller!" << endl;
					//		sol->print_solution();
					//		system("pause");
					//	}
					//}
				}
			}
			start_point = end_point + 1;
		}
	}
	return false;
}

bool SA::relocate_inter_fi(Solution* sol)
{
	//cout << "RelocateInter" << endl;

	for (int r1_index = 0; r1_index < data.numTruck; r1_index++)
	{
		int start_point_1 = 1;
		int end_point_1 = -1;
		for (int t1_index = 0; t1_index < data.numTripMax; t1_index++)
		{
			end_point_1 = start_point_1 + 1;

			// Find end of trip for route 1
			while (end_point_1 < sol->allroute[r1_index].size() && sol->allroute[r1_index][end_point_1].first != 0)
			{
				end_point_1++;
			}

			// Check if both start_point and end_point are 0
			if ((sol->allroute[r1_index][start_point_1].first == 0 && sol->allroute[r1_index][start_point_1 - 1].first == 0) || start_point_1 > sol->allroute[r1_index].size())
			{
				break;
			}
			for (int posi = start_point_1; posi < end_point_1; posi++)
			{
				for (int r2_index = 0; r2_index < data.numTruck; r2_index++)
				{
					// To make sure start_point_2's position is correct
					int start_point_2 = 1;

					for (int t2_index = 0; t2_index < data.numTripMax; t2_index++)
					{
						int end_point_2 = start_point_2 + 1;

						// Find end of trip for route 2
						while (end_point_2 < sol->allroute[r2_index].size() && sol->allroute[r2_index][end_point_2].first != 0)
						{
							end_point_2++;
						}

						// Check if both start_point_2 and end_point_2 are 0
						if (sol->allroute[r2_index][start_point_2].first == 0 && sol->allroute[r2_index][start_point_2 - 1].first == 0)
						{
							break;
						}

						for (int posj = start_point_2; posj <= end_point_2; posj++) // Include end_point_2 for insertion at the end
						{
							// Ensure posj is within bounds and not in the same trip for the same route
							if (posj > sol->allroute[r2_index].size()) break;
							if (r1_index == r2_index && posj >= start_point_1 && posj < end_point_1) continue;

							// Check if the element at posi is already in the trip of r2_index
							bool already_in_trip = false;
							for (int k = start_point_2; k < end_point_2; k++)
							{
								if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first)
								{
									already_in_trip = true;
									break;
								}
							}

							if (already_in_trip) continue;
							double pre_obj = sol->objfunc;
							bool pre_feasible = sol->isFeasible;
							double originobj = sol->objfunc;
							sol->relocate_a_bin(r1_index, posi, r2_index, posj);

							if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
							{
								return true;
							}
							else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
							{
								return true;
							}
							else
							{
								//RelocateInter different trip posi < posj
								if (r1_index == r2_index && posi < posj)
								{
									sol->relocate_a_bin(r2_index, posj - 1, r1_index, posi);
								}
								else if (r1_index == r2_index && posi > posj)
								{
									sol->relocate_a_bin(r2_index, posj, r1_index, posi + 1);
								}
								else
								{
									sol->relocate_a_bin(r2_index, posj, r1_index, posi);
								}
							}

							////RelocateInter different trip posi < posj
							//if (r1_index == r2_index && posi < posj )
							//{
							//	if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
							//	{
							//		pre_obj = sol->objfunc;
							//		//sol->print_solution();
							//		return true;
							//	}
							//	else
							//	{
							//		sol->relocate_a_bin(r2_index, posj - 1, r1_index, posi);
							//		//sol->print_routes();
							//		if (abs(sol->objfunc -  originobj) > 0.00001)
							//		{
							//			cout << "wrong here posi < posj!!" << endl << endl;
							//			sol->print_xvalues();
							//			sol->print_yvalues();
							//			sol->print_solution();
							//			system("pause");
							//		}
							//	}
							//}
							//else if (r1_index == r2_index && posi > posj)
							//{
							//	if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
							//	{
							//		pre_obj = sol->objfunc;
							//		//sol->print_solution();
							//		return true;
							//	}
							//	else
							//	{
							//		sol->relocate_a_bin(r2_index, posj, r1_index, posi + 1);
							//		//sol->print_routes();
							//		if (abs(sol->objfunc - originobj) > 0.00001)
							//		{
							//			cout << "wrong here posi > posj!!" << endl << endl;
							//			sol->print_xvalues();
							//			sol->print_yvalues();
							//			sol->print_solution();
							//			system("pause");
							//		}
							//	}
							//}
							//else
							//{
							//	if (sol->objfunc < pre_obj - ERROR || (sol->isFeasible && !pre_feasible))
							//	{
							//		pre_obj = sol->objfunc;
							//		//sol->print_solution();
							//		return true;
							//	}
							//	else
							//	{
							//		sol->relocate_a_bin(r2_index, posj, r1_index, posi);
							//		if (abs(sol->objfunc - originobj) > 0.00001)
							//		{
							//			cout << "wrong other condition!!!" << endl << endl;
							//			sol->print_xvalues();
							//			sol->print_yvalues();
							//			sol->print_solution();
							//			system("pause");
							//		}
							//	}
							//}
						}

						// Update start_point_2 for the next trip in route 2
						start_point_2 = end_point_2 + 1;
					}
				}
			}

			// Update start_point_1 for the next trip in route 1
			start_point_1 = end_point_1 + 1;
		}
	}
	return false;
}

bool SA::remove_fi(Solution* sol)
{
	//cout << "Remove_fi" << endl;

	// To record each bin's visit freq & times
	unordered_map<int, int> freq_map;
	unordered_map<int, int> route_count_map;

	for (int r = 0; r < sol->allroute.size(); r++)
	{
		unordered_map<int, bool> visited_in_route;
		for (const auto& node : sol->allroute[r])
		{
			freq_map[node.first]++;
			if (!visited_in_route[node.first] && node.first != 0)
			{
				route_count_map[node.first]++;
				visited_in_route[node.first] = true;
			}
		}
	}

	vector<int> potential_bin_remove;
	for (const auto& entry : route_count_map)
	{
		if (freq_map[entry.first] > 1) // Only consider nodes with frequency > 1
		{
			potential_bin_remove.push_back(entry.first);
		}
	}

	// Sort nodes to remove by frequency and remain_values
	sort(potential_bin_remove.begin(), potential_bin_remove.end(), [&sol, &freq_map](int a, int b) {
		if (freq_map[a] != freq_map[b])
			return freq_map[a] > freq_map[b];
		return sol->remain_values[a] < sol->remain_values[b];
		});

	if (potential_bin_remove.empty())
	{
		return false;
	}

	int node_to_remove = potential_bin_remove.front();

	int max_second = -1;
	int r_index = -1;
	int posi = -1;

	// Find the node with the maximum second freq
	for (int r = 0; r < sol->allroute.size(); r++)
	{
		for (int pos = 1; pos < sol->allroute[r].size(); pos++)
		{
			if (sol->allroute[r][pos].first == node_to_remove)
			{
				if (sol->allroute[r][pos].second > max_second)
				{
					max_second = sol->allroute[r][pos].second;
					r_index = r;
					posi = pos;
				}
			}
		}
	}

	//sol->print_routes();
	//cout << "Check route:" << r_index << " pos: " << posi << " bin:" << sol->allroute[r_index][posi].first << "_" << sol->allroute[r_index][posi].second << endl;
	pair<int, int> removed_bin = sol->allroute[r_index][posi]; // Save the removed bin info
	if (removed_bin.first != 0)
	{
		double pre_obj = sol->objfunc;
		bool pre_feasible = sol->isFeasible;

		sol->remove_a_bin(r_index, posi);
		if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
		{
			return true;
		}
		else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
		{
			return true;
		}
		else
		{
			sol->insert_a_bin(r_index, posi, removed_bin);
		}
	}
	return false;
}

bool SA::insert_fi(Solution* sol)
{
	//cout << "Insert_fi!!!" << endl;
	vector<vector<double>> potential_list = {};

	// Collect potential nodes that can be inserted
	for (int i = 1; i < data.numVertex; i++)
	{
		if (sol->num_visited[i] < data.numFreqMax && sol->remain_values[i] > data.upper_limit_waste)
		{
			potential_list.push_back({ (double)i, (double)sol->num_visited[i], sol->remain_values[i] });
		}
	}

	if (potential_list.empty())
	{
		return false;
	}

	// Sort potential_list based on remain_values in descending order
	sort(potential_list.begin(), potential_list.end(), [](const vector<double>& a, const vector<double>& b) {
		return a[2] > b[2];
		});

	//cout << "Potential nodes to be inserted: ";
	for (const auto& node_info : potential_list)
	{
		//cout << node_info[0] << " ";
	}
	//cout << endl;

	for (const auto& node_info : potential_list)
	{
		int node = (int)node_info[0];
		bool added = false;
		int next_freq = sol->num_visited[node] + 1;

		if (next_freq > data.numFreqMax - 1) {
			// If the next frequency exceeds the max frequency, skip this node
			continue;
		}

		for (int r = 0; r < sol->data.numTruck; r++)
		{
			for (int pos = 1; pos <= sol->allroute[r].size() - 1; pos++) // Allow insertion at the end too
			{
				pair<int, int> bin = { node, next_freq }; // Use the next frequency for insertion

				double pre_obj = sol->objfunc;
				bool pre_feasible = sol->isFeasible;
				double originobj = sol->objfunc;

				sol->insert_a_bin(r, pos, bin);

				if (!pre_feasible and (sol->objfunc < pre_obj - ERROR || sol->isFeasible))
				{
					return true;
				}
				else if (pre_feasible and (sol->objfunc < pre_obj - ERROR and sol->isFeasible))
				{
					return true;
				}
				else
				{
					sol->remove_a_bin(r, pos);
				}
			}
		}
	}
	return false;
}

bool SA::swap_intra_random(Solution* sol)
{
	//cout << "Swap_Intra_random" << endl;

	// Seed the random number generator with the current time

	// Randomly select a truck
	int r_index = std::rand() % data.numTruck;

	int start_point = 1;
	int end_point = -1;

	// Find the number of valid trips in the selected truck
	std::vector<std::pair<int, int>> trips;  // to store start and end points of each trip

	for (int t_index = 0; t_index < data.numTripMax; t_index++)
	{
		end_point = start_point + 1;

		// Find end of trip for the current route
		while (end_point < sol->allroute[r_index].size() && sol->allroute[r_index][end_point].first != 0)
		{
			end_point++;
		}

		// Check if both start_point and end_point are valid
		if (sol->allroute[r_index][start_point].first == 0 && sol->allroute[r_index][start_point - 1].first == 0)
		{
			break;
		}

		if (end_point - start_point > 1)
		{
			trips.push_back({ start_point, end_point });
		}

		// Update start_point for the next trip
		start_point = end_point + 1;
	}

	// Ensure there is at least one valid trip
	if (trips.empty())
	{
		return false;
	}

	// Randomly select a trip
	int trip_index = std::rand() % trips.size();
	start_point = trips[trip_index].first;
	end_point = trips[trip_index].second;

	// Randomly select two distinct positions within the trip
	int posi = start_point + std::rand() % (end_point - start_point);
	int posj = start_point + std::rand() % (end_point - start_point);

	// Ensure posi and posj are distinct
	while (posi == posj)
	{
		posj = start_point + std::rand() % (end_point - start_point);
	}

	double pre_obj = sol->objfunc;
	bool pre_feasible = sol->isFeasible;

	// Print the selected positions before swap
	//cout << "route: " << r_index << " initial pos i: " << posi << " initial pos j: " << posj << endl;

	// Swap the two positions
	if (sol->allroute[r_index][posi].first != 0 || sol->allroute[r_index][posj].first != 0)
	{
		//sol->print_routes();
		sol->swap_two_bin(r_index, posi, r_index, posj);
		//sol->print_routes();
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//cout << "check the cost after: " << pre_obj << endl;
			//system("pause");
			//sol->print_solution();
			return true;
		}
		else
		{
			// Swap back if no improvement or if solution becomes infeasible
			sol->swap_two_bin(r_index, posj, r_index, posi);
			//return true;
		}

	}
	return false;
}

bool SA::swap_intra_random2(Solution* sol)
{
	// berapa kali swap dicoba dalam sekali panggil
	const int max_swaps = 3;        // boleh kamu tune
	const int max_trials_per_swap = 20;

	// pilih satu route secara acak
	int r_index = std::rand() % data.numTruck;

	// kumpulkan daftar trip pada route ini (tanpa depot)
	int start_point = 1;
	int end_point = -1;
	std::vector<std::pair<int, int>> trips; // [start, end) end adalah index depot berikutnya

	for (int t = 0; t < data.numTripMax; ++t)
	{
		end_point = start_point + 1;
		while (end_point < (int)sol->allroute[r_index].size() &&
			sol->allroute[r_index][end_point].first != 0)
		{
			end_point++;
		}

		// kalau trip punya >= 2 node non-depot
		if (end_point - start_point > 1)
			trips.push_back({ start_point, end_point });

		// kalau start_point ketemu 0 di posisi ini, berarti habis
		if (start_point < (int)sol->allroute[r_index].size() &&
			sol->allroute[r_index][start_point].first == 0 &&
			sol->allroute[r_index][start_point - 1].first == 0)
		{
			break;
		}

		start_point = end_point + 1; // loncat depot
		if (start_point >= (int)sol->allroute[r_index].size()) break;
	}

	if (trips.empty()) return false;

	// pilih satu trip secara acak
	int trip_idx = std::rand() % trips.size();
	start_point = trips[trip_idx].first;
	end_point = trips[trip_idx].second;

	bool any_changed = false;

	for (int s = 0; s < max_swaps; ++s)
	{
		// cari dua posisi acak berbeda di dalam [start_point, end_point)
		int trials = 0;
		bool swapped_once = false;

		while (trials++ < max_trials_per_swap)
		{
			if (end_point - start_point <= 1) break;

			int posi = start_point + std::rand() % (end_point - start_point);
			int posj = start_point + std::rand() % (end_point - start_point);
			if (posi == posj) continue;

			// hindari depot
			if (sol->allroute[r_index][posi].first == 0 ||
				sol->allroute[r_index][posj].first == 0)
			{
				continue;
			}

			// lakukan swap
			sol->swap_two_bin(r_index, posi, r_index, posj);

			// terima kalau feasible (atau relaxed)
			if (sol->isFeasible || sol->isFeasible_relaxed)
			{
				any_changed = true;
				swapped_once = true;
				break; // lanjut ke swap berikutnya
			}
			else
			{
				// rollback
				sol->swap_two_bin(r_index, posj, r_index, posi);
			}
		}

		// kalau tak berhasil menemukan pasangan valid untuk swap ke-s, hentikan loop
		if (!swapped_once) break;
	}

	return any_changed;
}

bool SA::relocate_intra_random(Solution* sol)
{
	//cout << "Relocate_Intra_random" << endl;

	// Seed the random number generator with the current time

	// Randomly select a truck
	int r_index = std::rand() % data.numTruck;

	int start_point = 1;
	int end_point = -1;
	std::vector<std::pair<int, int>> trips;

	// Find all valid trips for the selected truck
	for (int t_index = 0; t_index < data.numTripMax; t_index++)
	{
		end_point = start_point + 1;
		while (end_point < sol->allroute[r_index].size() && sol->allroute[r_index][end_point].first != 0)
		{
			end_point++;
		}

		if (end_point - start_point > 1)
		{
			trips.push_back({ start_point, end_point });
		}
		start_point = end_point + 1;
	}

	// Ensure there is at least one valid trip
	if (trips.empty())
	{
		return false;
	}

	// Randomly select a trip
	int trip_index = std::rand() % trips.size();
	start_point = trips[trip_index].first;
	end_point = trips[trip_index].second;

	// Randomly select two distinct positions within the trip
	int posi = start_point + std::rand() % (end_point - start_point);
	int posj = start_point + std::rand() % (end_point - start_point);

	// Ensure posi and posj are distinct
	while (posi == posj)
	{
		posj = start_point + std::rand() % (end_point - start_point);
	}

	double pre_obj = sol->objfunc;
	bool pre_feasible = sol->isFeasible;

	// Print the selected positions before relocation
	//cout << "route: " << r_index << " posi: " << posi << " posj: " << posj << endl;

	// Relocate the bin
	if (sol->allroute[r_index][posi].first != 0)
	{
		sol->relocate_a_bin(r_index, posi, r_index, posj);
		//sol->print_routes();
		//system("pause");
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//cout << "Feasible!!" << endl;
			//sol->print_solution();
			return true;
		}
		else
		{
			// Relocate back if no improvement or if solution becomes infeasible
			//cout << "Infeasible!!" << endl;
			sol->relocate_a_bin(r_index, posj, r_index, posi);
			//sol->print_routes();
		}
	}
	return false;
}

bool SA::relocate_intra_random2(Solution* sol)
{
	//cout << "Relocate_Intra_random" << endl;
	relocate_intra_random(sol);

	// Seed the random number generator with the current time

	// Randomly select a truck
	int r_index = std::rand() % data.numTruck;

	int start_point = 1;
	int end_point = -1;
	std::vector<std::pair<int, int>> trips;

	// Find all valid trips for the selected truck
	for (int t_index = 0; t_index < data.numTripMax; t_index++)
	{
		end_point = start_point + 1;
		while (end_point < sol->allroute[r_index].size() && sol->allroute[r_index][end_point].first != 0)
		{
			end_point++;
		}

		if (end_point - start_point > 1)
		{
			trips.push_back({ start_point, end_point });
		}
		start_point = end_point + 1;
	}

	// Ensure there is at least one valid trip
	if (trips.empty())
	{
		return false;
	}

	// Randomly select a trip
	int trip_index = std::rand() % trips.size();
	start_point = trips[trip_index].first;
	end_point = trips[trip_index].second;

	// Randomly select two distinct positions within the trip
	int posi = start_point + std::rand() % (end_point - start_point);
	int posj = start_point + std::rand() % (end_point - start_point);

	// Ensure posi and posj are distinct
	while (posi == posj)
	{
		posj = start_point + std::rand() % (end_point - start_point);
	}

	double pre_obj = sol->objfunc;
	bool pre_feasible = sol->isFeasible;

	// Print the selected positions before relocation
	//cout << "route: " << r_index << " posi: " << posi << " posj: " << posj << endl;

	// Relocate the bin
	if (sol->allroute[r_index][posi].first != 0)
	{
		sol->relocate_a_bin(r_index, posi, r_index, posj);
		//sol->print_routes();
		//system("pause");
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//cout << "Feasible!!" << endl;
			//sol->print_solution();
			return true;
		}
		else
		{
			// Relocate back if no improvement or if solution becomes infeasible
			//cout << "Infeasible!!" << endl;
			sol->relocate_a_bin(r_index, posj, r_index, posi);
			//sol->print_routes();
		}
	}

	//ttSol->copy_from_sol(sol);

	//relocate_intra_random(ttSol);

	//if (ttSol < sol)
	//{		
	//	sol->copy_from_sol(ttSol);
	//}
	
	return false;	
	
}

bool SA::swap_inter_random(Solution* sol)
{
	//std::cout << "use random swap" << std::endl;

	// Seed the random number generator with the current time

	// Randomly select two distinct trucks
	int r1_index = std::rand() % data.numTruck;
	int r2_index = std::rand() % data.numTruck;
	while (r1_index == r2_index) {
		r2_index = std::rand() % data.numTruck;
	}

	int start_point_1 = 1, end_point_1 = -1, start_point_2 = 1, end_point_2 = -1;
	std::vector<std::pair<int, int>> trips1, trips2;

	// Find all valid trips for the first truck
	for (int t1_index = 0; t1_index < data.numTripMax; t1_index++) {
		end_point_1 = start_point_1 + 1;
		while (end_point_1 < sol->allroute[r1_index].size() - 1 && sol->allroute[r1_index][end_point_1].first != 0)
		{
			end_point_1++;
		}
		if (end_point_1 - start_point_1 > 1)
		{
			trips1.push_back({ start_point_1, end_point_1 });
		}
		start_point_1 = end_point_1 + 1;
	}

	// Find all valid trips for the second truck
	for (int t2_index = 0; t2_index < data.numTripMax; t2_index++) {
		end_point_2 = start_point_2 + 1;
		while (end_point_2 < sol->allroute[r2_index].size() && sol->allroute[r2_index][end_point_2].first != 0) {
			end_point_2++;
		}
		if (end_point_2 - start_point_2 > 1)
		{
			trips2.push_back({ start_point_2, end_point_2 });
		}
		start_point_2 = end_point_2 + 1;
	}

	// Ensure there is at least one valid trip in each truck
	if (trips1.empty() || trips2.empty()) {
		return false;
	}

	// Randomly select a trip from each truck
	int trip_index1 = std::rand() % trips1.size();
	int trip_index2 = std::rand() % trips2.size();
	start_point_1 = trips1[trip_index1].first;
	end_point_1 = trips1[trip_index1].second;
	start_point_2 = trips2[trip_index2].first;
	end_point_2 = trips2[trip_index2].second;

	// Randomly select two distinct positions within the trips
	int posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	int posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);

	// Ensure the positions are not zero and within the valid range
	while (sol->allroute[r1_index][posi].first == 0 && posi < end_point_1)
	{
		posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	}
	while (sol->allroute[r2_index][posj].first == 0 && posj < end_point_2)
	{
		posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);
	}

	// Check if posi or posj are invalid
	if (posi >= sol->allroute[r1_index].size() || posj >= sol->allroute[r2_index].size() ||
		sol->allroute[r1_index][posi].first == 0 || sol->allroute[r2_index][posj].first == 0)
	{
		return false;
	}

	double pre_obj = sol->objfunc;
	bool pre_feasible = sol->isFeasible;

	// Check if posi is already in the trip of r2_index or if posj is already in the trip of r1_index
	bool already_in_trip = false;
	for (int k = start_point_2; k < end_point_2; k++) {
		if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first) {
			already_in_trip = true;
			break;
		}
	}
	for (int k = start_point_1; k < end_point_1; k++) {
		if (sol->allroute[r1_index][k].first == sol->allroute[r2_index][posj].first) {
			already_in_trip = true;
			break;
		}
	}
	//cout << "route1: " << r1_index << " pos1:" << posi << " bin1:" << sol->allroute[r1_index][posi].first << "_" << sol->allroute[r1_index][posi].second << endl;
	//cout << "route2: " << r2_index << " pos2:" << posj << " bin2:" << sol->allroute[r2_index][posj].first << "_" << sol->allroute[r2_index][posj].second << endl;
	//system("pause");

	if (!already_in_trip and posi < sol->allroute[r1_index].size())
	{
		// Swap the two positions
		sol->swap_two_bin(r1_index, posi, r2_index, posj);
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//sol->print_solution();
			return true;
		}
		else
		{
			// Swap back if no improvement or if solution becomes infeasible
			sol->swap_two_bin(r2_index, posj, r1_index, posi);
		}
	}

	return false;
}

bool SA::swap_inter_random2(Solution* sol)
{
	const int max_swaps = 3;            // boleh kamu tune
	const int max_trials_per_swap = 20; // inter-route biasanya butuh trial lebih banyak

	// pilih dua route berbeda
	int r1_index = std::rand() % data.numTruck;
	int r2_index = std::rand() % data.numTruck;
	while (r2_index == r1_index) {
		r2_index = std::rand() % data.numTruck;
	}

	// kumpulkan trip pada r1
	int s1 = 1, e1 = -1;
	std::vector<std::pair<int, int>> trips1;
	for (int t = 0; t < data.numTripMax; ++t)
	{
		e1 = s1 + 1;
		while (e1 < (int)sol->allroute[r1_index].size() &&
			sol->allroute[r1_index][e1].first != 0)
		{
			e1++;
		}
		if (e1 - s1 > 1) trips1.push_back({ s1, e1 });
		s1 = e1 + 1;
		if (s1 >= (int)sol->allroute[r1_index].size()) break;
	}

	// kumpulkan trip pada r2
	int s2 = 1, e2 = -1;
	std::vector<std::pair<int, int>> trips2;
	for (int t = 0; t < data.numTripMax; ++t)
	{
		e2 = s2 + 1;
		while (e2 < (int)sol->allroute[r2_index].size() &&
			sol->allroute[r2_index][e2].first != 0)
		{
			e2++;
		}
		if (e2 - s2 > 1) trips2.push_back({ s2, e2 });
		s2 = e2 + 1;
		if (s2 >= (int)sol->allroute[r2_index].size()) break;
	}

	if (trips1.empty() || trips2.empty()) return false;

	// pilih masing-masing satu trip
	auto trip1 = trips1[std::rand() % trips1.size()];
	auto trip2 = trips2[std::rand() % trips2.size()];
	s1 = trip1.first; e1 = trip1.second;
	s2 = trip2.first; e2 = trip2.second;

	bool any_changed = false;

	for (int s = 0; s < max_swaps; ++s)
	{
		int trials = 0;
		bool swapped_once = false;

		while (trials++ < max_trials_per_swap)
		{
			if (e1 - s1 <= 1 || e2 - s2 <= 1) break;

			int posi = s1 + std::rand() % (e1 - s1);
			int posj = s2 + std::rand() % (e2 - s2);

			// skip depot
			if (sol->allroute[r1_index][posi].first == 0 ||
				sol->allroute[r2_index][posj].first == 0)
			{
				continue;
			}

			// jangan menukar jika node r1[posi] sudah ada di trip r2 (atau sebaliknya)
			bool already_in_trip = false;
			for (int k = s2; k < e2; ++k) {
				if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first) {
					already_in_trip = true; break;
				}
			}
			for (int k = s1; k < e1 && !already_in_trip; ++k) {
				if (sol->allroute[r1_index][k].first == sol->allroute[r2_index][posj].first) {
					already_in_trip = true; break;
				}
			}
			if (already_in_trip) continue;

			// lakukan swap
			sol->swap_two_bin(r1_index, posi, r2_index, posj);

			// terima bila feasible (atau relaxed)
			if (sol->isFeasible || sol->isFeasible_relaxed)
			{
				any_changed = true;
				swapped_once = true;
				break; // lanjut ke swap berikutnya
			}
			else
			{
				// rollback
				sol->swap_two_bin(r2_index, posj, r1_index, posi);
			}
		}

		// jika tidak dapat pasangan valid untuk swap ke-s, hentikan loop
		if (!swapped_once) break;
	}

	return any_changed;
}

bool SA::relocate_inter_random(Solution* sol)
{
	//cout << "Relocate_Inter_random" << endl;

	// Seed the random number generator with the current time

	// Randomly select two distinct trucks
	int r1_index = std::rand() % data.numTruck;
	int r2_index = std::rand() % data.numTruck;
	while (r1_index == r2_index)
	{
		r2_index = std::rand() % data.numTruck;
	}

	int start_point_1 = 1, end_point_1 = -1, start_point_2 = 1, end_point_2 = -1;
	std::vector<std::pair<int, int>> trips1, trips2;

	// Find all valid trips for the first truck
	for (int t1_index = 0; t1_index < data.numTripMax; t1_index++)
	{
		end_point_1 = start_point_1 + 1;
		while (end_point_1 < sol->allroute[r1_index].size() && sol->allroute[r1_index][end_point_1].first != 0)
		{
			end_point_1++;
		}
		if (end_point_1 - start_point_1 > 1)
		{
			trips1.push_back({ start_point_1, end_point_1 });
		}
		start_point_1 = end_point_1 + 1;
	}

	// Find all valid trips for the second truck
	for (int t2_index = 0; t2_index < data.numTripMax; t2_index++)
	{
		end_point_2 = start_point_2 + 1;
		while (end_point_2 < sol->allroute[r2_index].size() && sol->allroute[r2_index][end_point_2].first != 0)
		{
			end_point_2++;
		}
		if (end_point_2 - start_point_2 > 1)
		{
			trips2.push_back({ start_point_2, end_point_2 });
		}
		start_point_2 = end_point_2 + 1;
	}

	// Ensure there is at least one valid trip in each truck
	if (trips1.empty() || trips2.empty())
	{
		return false;
	}

	// Randomly select a trip from each truck
	int trip_index1 = std::rand() % trips1.size();
	int trip_index2 = std::rand() % trips2.size();
	start_point_1 = trips1[trip_index1].first;
	end_point_1 = trips1[trip_index1].second;
	start_point_2 = trips2[trip_index2].first;
	end_point_2 = trips2[trip_index2].second;

	// Randomly select a position within the trips
	int posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	int posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);

	// Ensure posi and posj are within bounds
	while (posi >= sol->allroute[r1_index].size())
	{
		posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	}
	while (posj >= sol->allroute[r2_index].size())
	{
		posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);
	}

	// Check if the element at posi is already in the trip of r2_index
	bool already_in_trip = false;
	for (int k = start_point_2; k < end_point_2; k++)
	{
		if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first)
		{
			already_in_trip = true;
			break;
		}
	}

	// Check if the element at posj is already in the trip of r1_index
	for (int k = start_point_1; k < end_point_1; k++)
	{
		if (sol->allroute[r1_index][k].first == sol->allroute[r2_index][posj].first)
		{
			already_in_trip = true;
			break;
		}
	}
	//cout << "route1: " << r1_index << " posi: " << posi << " route2: " << r2_index << " posj: " << posj << endl;
	if (!already_in_trip and sol->allroute[r1_index][posi].first != 0)
	{
		double pre_obj = sol->objfunc;
		bool pre_feasible = sol->isFeasible;

		// Print the selected positions before relocation
		//sol->print_routes();
		sol->relocate_a_bin(r1_index, posi, r2_index, posj);
		//sol->print_routes();
		//system("pause");
		if (sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//cout << "Feasible!!" << endl;
			//sol->print_solution();
			return true;
		}
		else
		{
			// Relocate back if no improvement or if solution becomes infeasible
			//cout << "Infeasible!!" << endl;
			//sol->print_routes();
			sol->relocate_a_bin(r2_index, posj, r1_index, posi);
		}
	}
	return false;
}

bool SA::relocate_inter_random2(Solution* sol)
{
	//cout << "Relocate_Inter_random" << endl;
	relocate_inter_random(sol);

	// Seed the random number generator with the current time

	// Randomly select two distinct trucks
	int r1_index = std::rand() % data.numTruck;
	int r2_index = std::rand() % data.numTruck;
	while (r1_index == r2_index)
	{
		r2_index = std::rand() % data.numTruck;
	}

	int start_point_1 = 1, end_point_1 = -1, start_point_2 = 1, end_point_2 = -1;
	std::vector<std::pair<int, int>> trips1, trips2;

	// Find all valid trips for the first truck
	for (int t1_index = 0; t1_index < data.numTripMax; t1_index++)
	{
		end_point_1 = start_point_1 + 1;
		while (end_point_1 < sol->allroute[r1_index].size() && sol->allroute[r1_index][end_point_1].first != 0)
		{
			end_point_1++;
		}
		if (end_point_1 - start_point_1 > 1)
		{
			trips1.push_back({ start_point_1, end_point_1 });
		}
		start_point_1 = end_point_1 + 1;
	}

	// Find all valid trips for the second truck
	for (int t2_index = 0; t2_index < data.numTripMax; t2_index++)
	{
		end_point_2 = start_point_2 + 1;
		while (end_point_2 < sol->allroute[r2_index].size() && sol->allroute[r2_index][end_point_2].first != 0)
		{
			end_point_2++;
		}
		if (end_point_2 - start_point_2 > 1)
		{
			trips2.push_back({ start_point_2, end_point_2 });
		}
		start_point_2 = end_point_2 + 1;
	}

	// Ensure there is at least one valid trip in each truck
	if (trips1.empty() || trips2.empty())
	{
		return false;
	}

	// Randomly select a trip from each truck
	int trip_index1 = std::rand() % trips1.size();
	int trip_index2 = std::rand() % trips2.size();
	start_point_1 = trips1[trip_index1].first;
	end_point_1 = trips1[trip_index1].second;
	start_point_2 = trips2[trip_index2].first;
	end_point_2 = trips2[trip_index2].second;

	// Randomly select a position within the trips
	int posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	int posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);

	// Ensure posi and posj are within bounds
	while (posi >= sol->allroute[r1_index].size())
	{
		posi = start_point_1 + std::rand() % (end_point_1 - start_point_1);
	}
	while (posj >= sol->allroute[r2_index].size())
	{
		posj = start_point_2 + std::rand() % (end_point_2 - start_point_2);
	}

	// Check if the element at posi is already in the trip of r2_index
	bool already_in_trip = false;
	for (int k = start_point_2; k < end_point_2; k++)
	{
		if (sol->allroute[r2_index][k].first == sol->allroute[r1_index][posi].first)
		{
			already_in_trip = true;
			break;
		}
	}

	// Check if the element at posj is already in the trip of r1_index
	for (int k = start_point_1; k < end_point_1; k++)
	{
		if (sol->allroute[r1_index][k].first == sol->allroute[r2_index][posj].first)
		{
			already_in_trip = true;
			break;
		}
	}
	//cout << "route1: " << r1_index << " posi: " << posi << " route2: " << r2_index << " posj: " << posj << endl;
	if (!already_in_trip and sol->allroute[r1_index][posi].first != 0)
	{
		double pre_obj = sol->objfunc;
		bool pre_feasible = sol->isFeasible;

		// Print the selected positions before relocation
		//sol->print_routes();
		sol->relocate_a_bin(r1_index, posi, r2_index, posj);
		//sol->print_routes();
		//system("pause");
		if (sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//cout << "Feasible!!" << endl;
			//sol->print_solution();
			return true;
		}
		else
		{
			// Relocate back if no improvement or if solution becomes infeasible
			//cout << "Infeasible!!" << endl;
			//sol->print_routes();
			sol->relocate_a_bin(r2_index, posj, r1_index, posi);
		}
	}

	//ttSol->copy_from_sol(sol);

	//relocate_inter_random(ttSol);

	//if (ttSol < sol)
	//{
	//	sol->copy_from_sol(ttSol);
	//}	

	return false;	
	
}

bool SA::remove_random(Solution* sol)
{
	// To record each bin's visit freq & times
	unordered_map<int, int> freq_map;
	unordered_map<int, int> route_count_map;

	for (int r = 0; r < sol->allroute.size(); r++)
	{
		unordered_map<int, bool> visited_in_route;
		for (const auto& node : sol->allroute[r])
		{
			freq_map[node.first]++;
			if (!visited_in_route[node.first] && node.first != 0)
			{
				route_count_map[node.first]++;
				visited_in_route[node.first] = true;
			}
		}
	}

	vector<int> potential_bin_remove;
	for (const auto& entry : route_count_map)
	{
		if (freq_map[entry.first] > 1)  // Only consider bins with visit frequency greater than 1
		{
			potential_bin_remove.push_back(entry.first);
		}
	}

	// Sort nodes to remove by frequency and remain_values
	sort(potential_bin_remove.begin(), potential_bin_remove.end(), [&sol, &freq_map](int a, int b) {
		if (freq_map[a] != freq_map[b])
			return freq_map[a] > freq_map[b];
		return sol->remain_values[a] < sol->remain_values[b];
		});

	if (potential_bin_remove.empty())
	{
		return false;
	}

	// Randomly select a node to remove
	int random_index = rand() % potential_bin_remove.size();
	int node_to_remove = potential_bin_remove[random_index];

	int max_second = -1;
	int r_index = -1;
	int posi = -1;

	// Find the node with the maximum second freq
	for (int r = 0; r < sol->allroute.size(); r++)
	{
		for (int pos = 1; pos < sol->allroute[r].size(); pos++)
		{
			if (sol->allroute[r][pos].first == node_to_remove)
			{
				if (sol->allroute[r][pos].second > max_second)
				{
					max_second = sol->allroute[r][pos].second;
					r_index = r;
					posi = pos;
				}
			}
		}
	}

	// Ensure that the first trip of the route has more than one point
	int trip_point_count = 0;
	for (int pos = 1; pos < sol->allroute[r_index].size(); pos++)
	{
		if (sol->allroute[r_index][pos].first != 0)
		{
			trip_point_count++;
		}
		else
		{
			break;
		}
	}

	//cout << "Remove the bin" << sol->allroute[r_index][posi].first <<"_" << sol->allroute[r_index][posi].second <<endl;
	//system("pause");
	pair<int, int> removed_bin = sol->allroute[r_index][posi]; // Save the removed bin info
	if (removed_bin.first != 0 && sol->allroute[r_index][posi].first != 0)
	{
		double pre_obj = sol->objfunc;
		bool pre_feasible = sol->isFeasible;
		sol->remove_a_bin(r_index, posi);
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//sol->print_solution();
			return true;
		}
		else
		{
			sol->insert_a_bin(r_index, posi, removed_bin);
		}
	}
	return false;
}

bool SA::remove_random2(Solution* sol)
{
	// To record each bin's visit freq & times
	unordered_map<int, int> freq_map;
	unordered_map<int, int> route_count_map;

	for (int r = 0; r < sol->allroute.size(); r++)
	{
		unordered_map<int, bool> visited_in_route;
		for (const auto& node : sol->allroute[r])
		{
			freq_map[node.first]++;
			if (!visited_in_route[node.first] && node.first != 0)
			{
				route_count_map[node.first]++;
				visited_in_route[node.first] = true;
			}
		}
	}

	vector<int> potential_bin_remove;
	for (const auto& entry : route_count_map)
	{
		if (freq_map[entry.first] > 1)  // Only consider bins with visit frequency greater than 1
		{
			potential_bin_remove.push_back(entry.first);
		}
	}

	// Sort nodes to remove by frequency and remain_values
	sort(potential_bin_remove.begin(), potential_bin_remove.end(), [&sol, &freq_map](int a, int b) {
		if (freq_map[a] != freq_map[b])
			return freq_map[a] > freq_map[b];
		return sol->remain_values[a] < sol->remain_values[b];
		});

	if (potential_bin_remove.empty())
	{
		return false;
	}

	// Randomly select a node to remove
	int random_index = rand() % potential_bin_remove.size();
	int node_to_remove = potential_bin_remove[random_index];

	int max_second = -1;
	int r_index = -1;
	int posi = -1;

	// Find the node with the maximum second freq
	for (int r = 0; r < sol->allroute.size(); r++)
	{
		for (int pos = 1; pos < sol->allroute[r].size(); pos++)
		{
			if (sol->allroute[r][pos].first == node_to_remove)
			{
				if (sol->allroute[r][pos].second > max_second)
				{
					max_second = sol->allroute[r][pos].second;
					r_index = r;
					posi = pos;
				}
			}
		}
	}

	// Ensure that the first trip of the route has more than one point
	int trip_point_count = 0;
	for (int pos = 1; pos < sol->allroute[r_index].size(); pos++)
	{
		if (sol->allroute[r_index][pos].first != 0)
		{
			trip_point_count++;
		}
		else
		{
			break;
		}
	}

	//cout << "Remove the bin" << sol->allroute[r_index][posi].first <<"_" << sol->allroute[r_index][posi].second <<endl;
	//system("pause");
	pair<int, int> removed_bin = sol->allroute[r_index][posi]; // Save the removed bin info
	if (removed_bin.first != 0 && sol->allroute[r_index][posi].first != 0)
	{
		double pre_obj = sol->objfunc;
		bool pre_feasible = sol->isFeasible;
		sol->remove_a_bin(r_index, posi);
		if (sol->isFeasible || sol->isFeasible_relaxed)
		{
			pre_obj = sol->objfunc;
			//sol->print_solution();
			return true;
		}
		else
		{
			sol->insert_a_bin(r_index, posi, removed_bin);
		}
	}
	
	//ttSol->copy_from_sol(sol);

	//remove_random(ttSol);

	//if (ttSol < sol)
	//{
	//	sol->copy_from_sol(ttSol);
	//}

	remove_random(sol);

	return false;	
	
}

bool SA::insert_random(Solution* sol)
{
	//cout << "Insert_random!!!" << endl;
	vector<vector<double>> potential_list = {};

	// Collect potential nodes that can be inserted
	for (int i = 1; i < data.numVertex; i++)
	{
		if (sol->num_visited[i] < data.numFreqMax && sol->remain_values[i] > data.upper_limit_waste)
		{
			potential_list.push_back({ (double)i, (double)sol->num_visited[i], sol->remain_values[i] });
		}
	}

	if (potential_list.empty())
	{
		return false;
	}

	int bin = potential_list[0][0];
	int bin_freq = iSol->num_visited[potential_list[0][0]] + 1;
	pair<int, int> potential_bin = make_pair(bin, bin_freq);

	//cout << "Potential nodes to be inserted: ";
	for (const auto& node_info : potential_list)
	{
		//cout << node_info[0] << " ";
	}
	//cout << endl;

	for (const auto& node_info : potential_list)
	{
		int node = (int)node_info[0];
		bool added = false;
		int max_freq = sol->num_visited[node] + 1;

		for (int r = 0; r < sol->data.numTruck; r++)
		{
			for (int pos = 1; pos <= sol->allroute[r].size(); pos++) // Allow insertion at the end too
			{
				// Ensure the node has not reached max visit frequency
				if (sol->num_visited[node] >= data.numFreqMax - 1 || pos >= sol->allroute[r].size())
				{
					//cout << "Node " << node << " has reached maximum visit frequency." << endl;
					continue;
				}

				pair<int, int> bin = { node, max_freq }; // Use the highest frequency for insertion

				double pre_obj = sol->objfunc;
				bool pre_feasible = sol->isFeasible;
				//bool originobj = sol->isFeasible;

				//cout << "Before insert:" << endl;
				/*sol->print_solution();
				sol->print_xvalues();
				sol->print_yvalues();
				cout << "Check the route: " << r << " pos: " << pos << " bin: " << bin.first << "_" << bin.second << endl;*/

				sol->insert_a_bin(r, pos, bin);

				//cout << "After insert:" << endl;
				//sol->print_routes();

				if (sol->isFeasible || sol->isFeasible_relaxed)
				{
					//cout << "Successfully added node " << node << " to route " << r << " at position " << pos << endl;
					return true;
				}
				else
				{
					sol->remove_a_bin(r, pos);
					//if (abs(sol->objfunc - originobj) > 0.00001)
					//{
					//	//sol->print_xvalues();
					//	//sol->print_yvalues();
					//	cout << "wrong here insert!!!" << endl;
					//	sol->print_solution();
					//	sol->print_xvalues();
					//	sol->print_yvalues();
					//	system("pause");
					//}
				}
			}
		}
	}

	return false;
}

bool SA::insert_random2(Solution* sol)
{
	//cout << "Insert_random!!!" << endl;
	vector<vector<double>> potential_list = {};

	// Collect potential nodes that can be inserted
	for (int i = 1; i < data.numVertex; i++)
	{
		if (sol->num_visited[i] < data.numFreqMax && sol->remain_values[i] > data.upper_limit_waste)
		{
			potential_list.push_back({ (double)i, (double)sol->num_visited[i], sol->remain_values[i] });
		}
	}

	if (potential_list.empty())
	{
		return false;
	}

	int bin = potential_list[0][0];
	int bin_freq = iSol->num_visited[potential_list[0][0]] + 1;
	pair<int, int> potential_bin = make_pair(bin, bin_freq);

	//cout << "Potential nodes to be inserted: ";
	for (const auto& node_info : potential_list)
	{
		//cout << node_info[0] << " ";
	}
	//cout << endl;

	for (const auto& node_info : potential_list)
	{
		int node = (int)node_info[0];
		bool added = false;
		int max_freq = sol->num_visited[node] + 1;

		for (int r = 0; r < sol->data.numTruck; r++)
		{
			for (int pos = 1; pos <= sol->allroute[r].size(); pos++) // Allow insertion at the end too
			{
				// Ensure the node has not reached max visit frequency
				if (sol->num_visited[node] >= data.numFreqMax - 1 || pos >= sol->allroute[r].size())
				{
					//cout << "Node " << node << " has reached maximum visit frequency." << endl;
					continue;
				}

				pair<int, int> bin = { node, max_freq }; // Use the highest frequency for insertion

				double pre_obj = sol->objfunc;
				bool pre_feasible = sol->isFeasible;
				//bool originobj = sol->isFeasible;

				//cout << "Before insert:" << endl;
				/*sol->print_solution();
				sol->print_xvalues();
				sol->print_yvalues();
				cout << "Check the route: " << r << " pos: " << pos << " bin: " << bin.first << "_" << bin.second << endl;*/

				sol->insert_a_bin(r, pos, bin);

				//cout << "After insert:" << endl;
				//sol->print_routes();

				if (sol->isFeasible || sol->isFeasible_relaxed)
				{
					//cout << "Successfully added node " << node << " to route " << r << " at position " << pos << endl;
					return true;
				}
				else
				{
					sol->remove_a_bin(r, pos);
					//if (abs(sol->objfunc - originobj) > 0.00001)
					//{
					//	//sol->print_xvalues();
					//	//sol->print_yvalues();
					//	cout << "wrong here insert!!!" << endl;
					//	sol->print_solution();
					//	sol->print_xvalues();
					//	sol->print_yvalues();
					//	system("pause");
					//}
				}
			}
		}
	}

	//ttSol->copy_from_sol(sol);

	//insert_random(ttSol);

	//if (ttSol < sol)
	//{
	//	sol->copy_from_sol(ttSol);
	//}

	insert_random(sol);

	return false;	
	
}

bool SA::two_opt_intra(Solution* sol) {
	for (int r = 0; r < data.numTruck; r++) {
		// telusuri per-trip (hindari depot 0)
		int s = 1;
		while (s < (int)sol->allroute[r].size() && sol->allroute[r][s].first != 0) {
			// cari akhir trip
			int e = s + 1;
			while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;

			for (int i = s; i < e - 1; i++) {
				for (int j = i + 1; j < e; j++) {
					// backup segmen
					auto before = sol->allroute[r];

					// reverse segmen [i..j] **dengan seluruh pair**
					std::reverse(sol->allroute[r].begin() + i, sol->allroute[r].begin() + j + 1);

					double pre_obj = sol->objfunc;
					bool pre_feas = sol->isFeasible;
					sol->solve_masterproblem();

					if (sol->objfunc < pre_obj - ERROR || (!pre_feas && sol->isFeasible)) {
						return true;
					}
					else {
						sol->allroute[r] = std::move(before); // revert
						sol->solve_masterproblem();
					}
				}
			}
			s = e + 1; // loncat ke trip berikutnya
		}
	}
	return false;
}

bool SA::two_opt_intra2(Solution* sol)
{
	// Loop through each route (r_index)
	for (int r_index = 0; r_index < data.numTruck; r_index++)
	{
		// Loop through each node i in the route starting from 1 (to skip depot 0)
		for (int i = 1; i < sol->allroute[r_index].size() - 1; i++)
		{
			// Loop through each node j after node i
			for (int j = i + 1; j < sol->allroute[r_index].size(); j++)
			{
				// Make sure i and j are not the depot (0)
				if (sol->allroute[r_index][i].first == 0 || sol->allroute[r_index][j].first == 0)
					continue;

				// Calculate the cost before the swap
				double pre_obj = sol->objfunc;
				bool pre_feasible = sol->isFeasible;

				// Perform the 2-opt swap: reverse the segment between i and j
				for (int k = 0; k <= (j - i) / 2; k++)
				{
					int temp = sol->allroute[r_index][i + k].first;
					sol->allroute[r_index][i + k].first = sol->allroute[r_index][j - k].first;
					sol->allroute[r_index][j - k].first = temp;
				}

				// Recompute the master problem and the cost
				sol->solve_masterproblem();

				// If the new cost is lower, accept the solution and stop further iterations
				if (sol->objfunc < pre_obj || (sol->isFeasible && !pre_feasible))
				{
					return true;  // Improvement found, accept the move
				}
				else
				{
					// Revert the swap if no improvement
					for (int k = 0; k <= (j - i) / 2; k++)
					{
						int temp = sol->allroute[r_index][i + k].first;
						sol->allroute[r_index][i + k].first = sol->allroute[r_index][j - k].first;
						sol->allroute[r_index][j - k].first = temp;
					}
				}
			}
		}
	}

	ttSol->copy_from_sol(sol);

	two_opt_intra(ttSol);

	if (ttSol < sol)
	{
		sol->copy_from_sol(ttSol);
	}

	// If no improvement is found, return false
	return false;
}

bool SA::three_opt_intra(Solution* sol)
{
	//cout << "three_opt_intra" << endl;
	auto revcat = [](const std::vector<std::pair<int, int>>& v) {
		std::vector<std::pair<int, int>> w = v;
		std::reverse(w.begin(), w.end());
		return w;
	};

	for (int r = 0; r < data.numTruck; ++r) {
		int start = 1;
		while (start < (int)sol->allroute[r].size()) {

			// temukan akhir trip (index depot selanjutnya)
			int end = start;
			while (end < (int)sol->allroute[r].size() && sol->allroute[r][end].first != 0) end++;

			// trip minimal 4 node non-depot agar ada i<j<k valid
			if (end - start >= 4) {
				// Enumerasi breakpoint i<j<k
				for (int i = start; i <= end - 4; ++i) {
					for (int j = i + 1; j <= end - 3; ++j) {
						for (int k = j + 1; k <= end - 2; ++k) {

							// backup route r
							auto route_backup = sol->allroute[r];
							double pre_obj = sol->objfunc;
							bool pre_feas = sol->isFeasible;

							// potong segmen di dalam trip
							std::vector<std::pair<int, int>> S1, S2, S3, S4;
							S1.assign(sol->allroute[r].begin() + start, sol->allroute[r].begin() + i + 1);
							S2.assign(sol->allroute[r].begin() + i + 1, sol->allroute[r].begin() + j + 1);
							S3.assign(sol->allroute[r].begin() + j + 1, sol->allroute[r].begin() + k + 1);
							S4.assign(sol->allroute[r].begin() + k + 1, sol->allroute[r].begin() + end);

							// daftar 7 kasus
							const int CASES = 7;
							for (int c = 0; c < CASES; ++c) {
								// bangun kandidat trip baru
								std::vector<std::pair<int, int>> trip_new;
								trip_new.reserve(end - start);

								switch (c) {
								case 0: // S1 + rev(S2) + S3 + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									{ auto t = revcat(S2); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S3.begin(), S3.end());
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 1: // S1 + S2 + rev(S3) + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									trip_new.insert(trip_new.end(), S2.begin(), S2.end());
									{ auto t = revcat(S3); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 2: // S1 + rev(S2) + rev(S3) + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									{ auto t = revcat(S2); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									{ auto t = revcat(S3); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 3: // S1 + S3 + S2 + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									trip_new.insert(trip_new.end(), S3.begin(), S3.end());
									trip_new.insert(trip_new.end(), S2.begin(), S2.end());
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 4: // S1 + rev(S3) + S2 + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									{ auto t = revcat(S3); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S2.begin(), S2.end());
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 5: // S1 + S3 + rev(S2) + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									trip_new.insert(trip_new.end(), S3.begin(), S3.end());
									{ auto t = revcat(S2); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;

								case 6: // S1 + rev(S3) + rev(S2) + S4
									trip_new.insert(trip_new.end(), S1.begin(), S1.end());
									{ auto t = revcat(S3); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									{ auto t = revcat(S2); trip_new.insert(trip_new.end(), t.begin(), t.end()); }
									trip_new.insert(trip_new.end(), S4.begin(), S4.end());
									break;
								}

								// tulis balik hanya bagian trip [start..end-1]
								for (int t = 0; t < (int)trip_new.size(); ++t) {
									sol->allroute[r][start + t] = trip_new[t];
								}

								sol->solve_masterproblem();

								if (sol->isFeasible && (sol->objfunc < pre_obj - ERROR || !pre_feas)) {
									return true; // first-improvement
								}

								// rollback untuk coba kasus lain
								sol->allroute[r] = route_backup;
							} // end for CASES
						} // k
					} // j
				} // i
			}
			// lanjut ke trip berikutnya
			start = end + 1;
		}
	}
	return false;
}

bool SA::swap22_intra_random(Solution* sol)
{
	int r = std::rand() % data.numTruck;

	// enumerasi trip dalam rute r
	int s = 1;
	while (s < (int)sol->allroute[r].size() && sol->allroute[r][s].first != 0) {
		int e = s + 1;
		while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;

		int len = e - s;
		if (len >= 4) {
			// pilih dua start blok berbeda (berurutan 2-2) dan tidak overlap
			int i = s + (std::rand() % (len - 1 - 1));            // i in [s, e-2]
			int j = s + (std::rand() % (len - 1 - 1));            // j in [s, e-2]
			if (j == i || j == i + 1) j = (j + 2 <= e - 2) ? j + 2 : s; // hindari overlap sederhana
			if (i > j) std::swap(i, j);

			// backup dan tukar blok
			auto before = sol->allroute[r];
			std::swap(sol->allroute[r][i], sol->allroute[r][j]);
			std::swap(sol->allroute[r][i + 1], sol->allroute[r][j + 1]);

			double pre = sol->objfunc; bool feas = sol->isFeasible;
			sol->solve_masterproblem();
			if (sol->objfunc < pre - ERROR || (!feas && sol->isFeasible)) return true;

			// revert
			sol->allroute[r] = std::move(before);
			sol->solve_masterproblem();
		}
		s = e + 1;
	}
	return false;
}

bool SA::swap22_inter_random(Solution* sol) 
{
	int r1 = std::rand() % data.numTruck;
	int r2 = std::rand() % data.numTruck;
	if (r1 == r2) return false;

	// ambil satu trip acak per rute
	auto pick_trip = [&](int r, int& s, int& e)->bool {
		s = 1;
		std::vector<std::pair<int, int>> spans;
		while (s < (int)sol->allroute[r].size() && sol->allroute[r][s].first != 0) {
			e = s + 1;
			while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;
			if (e - s >= 2) spans.push_back({ s,e });
			s = e + 1;
		}
		if (spans.empty()) return false;
		auto sp = spans[std::rand() % spans.size()];
		s = sp.first; e = sp.second; return true;
	};

	int s1, e1, s2, e2;
	if (!pick_trip(r1, s1, e1) || !pick_trip(r2, s2, e2)) return false;
	if (e1 - s1 < 2 || e2 - s2 < 2) return false;

	int i = s1 + (std::rand() % (e1 - s1 - 1)); // blok i,i+1
	int j = s2 + (std::rand() % (e2 - s2 - 1)); // blok j,j+1

	auto before1 = sol->allroute[r1];
	auto before2 = sol->allroute[r2];

	std::swap(sol->allroute[r1][i], sol->allroute[r2][j]);
	std::swap(sol->allroute[r1][i + 1], sol->allroute[r2][j + 1]);

	double pre = sol->objfunc; bool feas = sol->isFeasible;
	sol->solve_masterproblem();
	if (sol->objfunc < pre - ERROR || (!feas && sol->isFeasible)) return true;

	sol->allroute[r1] = std::move(before1);
	sol->allroute[r2] = std::move(before2);
	sol->solve_masterproblem();
	return false;
}

bool SA::shift20_intra_random(Solution* sol)
{
	int r = std::rand() % data.numTruck;

	int s = 1;
	while (s < (int)sol->allroute[r].size() && sol->allroute[r][s].first != 0) {
		int e = s + 1;
		while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;

		if (e - s >= 3) {
			int i = s + (std::rand() % (e - s - 1)); // start blok
			if (i + 1 >= e) { s = e + 1; continue; }

			int insertPos = s + (std::rand() % (e - s - 1)); // posisi baru
			// pastikan tidak menyisipkan di dalam blok asal
			if (insertPos >= i && insertPos <= i + 1) {
				insertPos = (i + 2 < e) ? i + 2 : s;
			}

			auto before = sol->allroute[r];
			// ambil blok
			auto a = sol->allroute[r][i];
			auto b = sol->allroute[r][i + 1];
			// hapus dari index lebih besar dahulu
			sol->allroute[r].erase(sol->allroute[r].begin() + i, sol->allroute[r].begin() + i + 2);

			// penyesuaian posisi sisip bila melewati penghapusan
			if (insertPos > i) insertPos -= 2;

			sol->allroute[r].insert(sol->allroute[r].begin() + insertPos, b);
			sol->allroute[r].insert(sol->allroute[r].begin() + insertPos, a);

			double pre = sol->objfunc; bool feas = sol->isFeasible;
			sol->solve_masterproblem();
			if (sol->objfunc < pre - ERROR || (!feas && sol->isFeasible)) return true;

			sol->allroute[r] = std::move(before);
			sol->solve_masterproblem();
		}
		s = e + 1;
	}
	return false;
}

bool SA::shift20_inter_random(Solution* sol)
 {
	int r1 = std::rand() % data.numTruck;
	int r2 = std::rand() % data.numTruck;
	if (r1 == r2) return false;

	auto pick_trip = [&](int r, int& s, int& e)->bool {
		s = 1;
		std::vector<std::pair<int, int>> spans;
		while (s < (int)sol->allroute[r].size() && sol->allroute[r][s].first != 0) {
			e = s + 1;
			while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;
			if (e - s >= 2) spans.push_back({ s,e });
			s = e + 1;
		}
		if (spans.empty()) return false;
		auto sp = spans[std::rand() % spans.size()];
		s = sp.first; e = sp.second; return true;
	};

	int s1, e1, s2, e2;
	if (!pick_trip(r1, s1, e1) || !pick_trip(r2, s2, e2)) return false;

	int i = s1 + (std::rand() % (e1 - s1 - 1));
	if (i + 1 >= e1) return false;

	int insertPos = s2 + (std::rand() % (e2 - s2 + 1)); // boleh di ujung

	auto before1 = sol->allroute[r1];
	auto before2 = sol->allroute[r2];

	auto a = sol->allroute[r1][i];
	auto b = sol->allroute[r1][i + 1];
	sol->allroute[r1].erase(sol->allroute[r1].begin() + i, sol->allroute[r1].begin() + i + 2);

	sol->allroute[r2].insert(sol->allroute[r2].begin() + insertPos, b);
	sol->allroute[r2].insert(sol->allroute[r2].begin() + insertPos, a);

	double pre = sol->objfunc; bool feas = sol->isFeasible;
	sol->solve_masterproblem();
	if (sol->objfunc < pre - ERROR || (!feas && sol->isFeasible)) return true;

	sol->allroute[r1] = std::move(before1);
	sol->allroute[r2] = std::move(before2);
	sol->solve_masterproblem();
	return false;
}

bool SA::swap22_intra_fi(Solution* sol)
{
	for (int r = 0; r < data.numTruck; r++) {
		// telusuri tiap trip pada rute r
		int s = 1;
		while (s < (int)sol->allroute[r].size()) {
			if (sol->allroute[r][s].first == 0) { s++; continue; }
			int e = s + 1;
			while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;

			// panjang trip minimal 4 untuk swap(2,2)
			if (e - s >= 4) {
				for (int i = s; i <= e - 2; ++i) {                  // blok1: (i,i+1)
					if (sol->allroute[r][i].first == 0 || sol->allroute[r][i + 1].first == 0) continue;
					for (int j = i + 2; j <= e - 2; ++j) {          // blok2: (j,j+1), non-overlap
						if (sol->allroute[r][j].first == 0 || sol->allroute[r][j + 1].first == 0) continue;

						auto before = sol->allroute[r];
						double pre_obj = sol->objfunc;
						bool   pre_feas = sol->isFeasible;

						// tukar dua pasang berurutan
						std::swap(sol->allroute[r][i], sol->allroute[r][j]);
						std::swap(sol->allroute[r][i + 1], sol->allroute[r][j + 1]);

						sol->solve_masterproblem();
						if ((!pre_feas && (sol->objfunc < pre_obj - ERROR || sol->isFeasible)) ||
							(pre_feas && (sol->objfunc < pre_obj - ERROR && sol->isFeasible))) {
							return true; // first-improvement
						}
						// revert
						sol->allroute[r] = std::move(before);
						sol->solve_masterproblem();
					}
				}
			}
			s = e + 1; // next trip
		}
	}
	return false;
}

bool SA::swap22_inter_fi(Solution* sol)
{
	for (int r1 = 0; r1 < data.numTruck; ++r1) {
		// daftar trip di r1
		std::vector<std::pair<int, int>> trips1;
		int s1 = 1, e1;
		while (s1 < (int)sol->allroute[r1].size()) {
			if (sol->allroute[r1][s1].first == 0) { s1++; continue; }
			e1 = s1 + 1;
			while (e1 < (int)sol->allroute[r1].size() && sol->allroute[r1][e1].first != 0) e1++;
			if (e1 - s1 >= 2) trips1.push_back({ s1,e1 });
			s1 = e1 + 1;
		}
		if (trips1.empty()) continue;

		for (int r2 = 0; r2 < data.numTruck; ++r2) {
			if (r2 == r1) continue; // inter-route murni; jika mau antar-trip dalam rute sama, hapus baris ini

			// daftar trip di r2
			std::vector<std::pair<int, int>> trips2;
			int s2 = 1, e2;
			while (s2 < (int)sol->allroute[r2].size()) {
				if (sol->allroute[r2][s2].first == 0) { s2++; continue; }
				e2 = s2 + 1;
				while (e2 < (int)sol->allroute[r2].size() && sol->allroute[r2][e2].first != 0) e2++;
				if (e2 - s2 >= 2) trips2.push_back({ s2,e2 });
				s2 = e2 + 1;
			}
			if (trips2.empty()) continue;

			for (const auto& trip1 : trips1) 
			{
				for (const auto& trip2 : trips2) 
				{
					int a = trip1.first;   // start index of trip 1
					int b = trip1.second;  // end index of trip 1
					int c = trip2.first;   // start index of trip 2
					int d = trip2.second;  // end index of trip 2
					for (int i = a; i <= b - 2; ++i) {
						if (sol->allroute[r1][i].first == 0 || sol->allroute[r1][i + 1].first == 0) continue;
						for (int j = c; j <= d - 2; ++j) {
							if (sol->allroute[r2][j].first == 0 || sol->allroute[r2][j + 1].first == 0) continue;

							auto before1 = sol->allroute[r1];
							auto before2 = sol->allroute[r2];
							double pre_obj = sol->objfunc;
							bool   pre_feas = sol->isFeasible;

							std::swap(sol->allroute[r1][i], sol->allroute[r2][j]);
							std::swap(sol->allroute[r1][i + 1], sol->allroute[r2][j + 1]);

							sol->solve_masterproblem();
							if ((!pre_feas && (sol->objfunc < pre_obj - ERROR || sol->isFeasible)) ||
								(pre_feas && (sol->objfunc < pre_obj - ERROR && sol->isFeasible))) {
								return true;
							}
							// revert
							sol->allroute[r1] = std::move(before1);
							sol->allroute[r2] = std::move(before2);
							sol->solve_masterproblem();
						}
					}
				}
			}
		}
	}
	return false;
}

bool SA::shift20_intra_fi(Solution* sol)
{
	for (int r = 0; r < data.numTruck; ++r) {
		int s = 1;
		while (s < (int)sol->allroute[r].size()) {
			if (sol->allroute[r][s].first == 0) { s++; continue; }
			int e = s + 1;
			while (e < (int)sol->allroute[r].size() && sol->allroute[r][e].first != 0) e++;

			if (e - s >= 3) { // butuh minimal 3 node untuk pindah blok 2
				for (int i = s; i <= e - 2; ++i) {        // blok asal (i,i+1)
					if (sol->allroute[r][i].first == 0 || sol->allroute[r][i + 1].first == 0) continue;

					for (int ins = s; ins <= e; ++ins) {  // posisi sisip sebelum indeks ins (boleh e = sebelum depot)
						if (ins >= i && ins <= i + 1) continue; // jangan sisip di dalam blok asal

						auto before = sol->allroute[r];
						double pre_obj = sol->objfunc;
						bool   pre_feas = sol->isFeasible;

						// ambil blok 2
						auto a = sol->allroute[r][i];
						auto b = sol->allroute[r][i + 1];
						sol->allroute[r].erase(sol->allroute[r].begin() + i, sol->allroute[r].begin() + i + 2);

						int insAdj = ins;
						if (insAdj > i) insAdj -= 2; // koreksi pergeseran setelah erase

						// sisip urutan sama: a lalu b
						sol->allroute[r].insert(sol->allroute[r].begin() + insAdj, a);
						sol->allroute[r].insert(sol->allroute[r].begin() + insAdj + 1, b);

						sol->solve_masterproblem();
						if ((!pre_feas && (sol->objfunc < pre_obj - ERROR || sol->isFeasible)) ||
							(pre_feas && (sol->objfunc < pre_obj - ERROR && sol->isFeasible))) {
							return true;
						}
						// revert
						sol->allroute[r] = std::move(before);
						sol->solve_masterproblem();
					}
				}
			}
			s = e + 1;
		}
	}
	return false;
}

bool SA::shift20_inter_fi(Solution* sol)
{
	for (int r1 = 0; r1 < data.numTruck; ++r1) 
	{
		// trip pada r1
		std::vector<std::pair<int, int>> trips1;
		int s1 = 1, e1;
		while (s1 < (int)sol->allroute[r1].size())
		{
			if (sol->allroute[r1][s1].first == 0) { s1++; continue; }
			e1 = s1 + 1;
			while (e1 < (int)sol->allroute[r1].size() && sol->allroute[r1][e1].first != 0) e1++;
			if (e1 - s1 >= 2) trips1.push_back({ s1,e1 });
			s1 = e1 + 1;
		}
		if (trips1.empty()) continue;

		for (int r2 = 0; r2 < data.numTruck; ++r2) 
		{
			if (r2 == r1) continue;

			// trip pada r2
			std::vector<std::pair<int, int>> trips2;
			int s2 = 1, e2;
			while (s2 < (int)sol->allroute[r2].size()) 
			{
				if (sol->allroute[r2][s2].first == 0) { s2++; continue; }
				e2 = s2 + 1;
				while (e2 < (int)sol->allroute[r2].size() && sol->allroute[r2][e2].first != 0) e2++;
				trips2.push_back({ s2,e2 }); // boleh panjang 0/1, sisip tetap valid di tepi
				s2 = e2 + 1;
			}
			if (trips2.empty()) continue;

			for (const auto& trip1 : trips1) 
			{
				for (const auto& trip2 : trips2) 
				{
					int a = trip1.first;   // start index of trip 1
					int b = trip1.second;  // end index of trip 1
					int c = trip2.first;   // start index of trip 2
					int d = trip2.second;  // end index of trip 2
				
				for (int i = a; i <= b - 2; ++i) 
				{
					if (sol->allroute[r1][i].first == 0 || sol->allroute[r1][i + 1].first == 0) continue;

					for (int ins = c; ins <= d; ++ins) 
					{
						auto before1 = sol->allroute[r1];
						auto before2 = sol->allroute[r2];
						double pre_obj = sol->objfunc;
						bool   pre_feas = sol->isFeasible;

						auto a1 = sol->allroute[r1][i];
						auto a2 = sol->allroute[r1][i + 1];
						// hapus dari r1
						sol->allroute[r1].erase(sol->allroute[r1].begin() + i, sol->allroute[r1].begin() + i + 2);

						// sisip ke r2 (urutan dipertahankan)
						sol->allroute[r2].insert(sol->allroute[r2].begin() + ins, a1);
						sol->allroute[r2].insert(sol->allroute[r2].begin() + ins + 1, a2);
						
						sol->solve_masterproblem();
						if ((!pre_feas && (sol->objfunc < pre_obj - ERROR || sol->isFeasible)) ||
							(pre_feas && (sol->objfunc < pre_obj - ERROR && sol->isFeasible))) {
							return true;
						}
						// revert
						sol->allroute[r1] = std::move(before1);
						sol->allroute[r2] = std::move(before2);
						sol->solve_masterproblem();
					}
				}
			}
		}
	}
}
	return false;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
bool SA::relocation_heuristic(Solution* sol)
{
	// Loop through each truck (route)
	for (int r_index = 0; r_index < data.numTruck; r_index++)
	{
		int start_point = 1;
		int end_point = -1;

		// Traverse each route
		for (int t_index = 0; t_index < data.numTripMax; t_index++)
		{
			end_point = start_point + 1;
			while (sol->allroute[r_index][end_point].first != 0) end_point++;

			// Try relocating segments of the route
			for (int i = start_point; i < end_point; i++)
			{
				for (int j = i + 1; j < end_point; j++)
				{
					// Calculate cost improvement for relocation
					double pre_cost = sol->objfunc;
					bool pre_feasible = sol->isFeasible;

					// Perform relocation
					sol->relocate_a_bin(r_index, i, r_index, j);

					// Check if it improves the solution
					if (sol->objfunc < pre_cost - ERROR || (sol->isFeasible && !pre_feasible))
					{
						return true;
					}
					else
					{
						// Revert if no improvement
						sol->relocate_a_bin(r_index, j, r_index, i);
					}
				}
			}

			start_point = end_point + 1;
		}
	}
	return false;
}
*/

/*
bool SA::three_opt_intra(Solution* sol)
{
	bool isImproved = false;

	// Iterate through all routes
	for (int r_index = 0; r_index < data.numTruck; r_index++)
	{
		int start_point = 1;   // Start from the first node (avoid depot node 0)
		int end_point = -1;

		for (int t_index = 0; t_index < data.numTripMax; t_index++)
		{
			// Find the end of the trip for the route
			end_point = start_point + 1;
			while (sol->allroute[r_index][end_point].first != 0)
			{
				end_point++;
			}

			// Ensure there are at least 3 nodes in the trip (excluding depot)
			if (end_point - start_point > 3)
			{
				for (int i = start_point; i < end_point - 2; i++) // i is the first of the 3 consecutive nodes
				{
					for (int j = i + 1; j < end_point - 1; j++) // j is the second node of the block
					{
						for (int k = j + 1; k < end_point; k++) // k is the third node of the block
						{
							// Save the original block of 3 nodes
							pair<int, int> node1 = sol->allroute[r_index][i];
							pair<int, int> node2 = sol->allroute[r_index][j];
							pair<int, int> node3 = sol->allroute[r_index][k];

							// Ensure that we're not removing the depot node
							if (node1.first == 0 || node2.first == 0 || node3.first == 0) {
								continue; // Skip if depot is part of the block
							}

							// Remove the 3-block from the route
							sol->remove_a_bin(r_index, i);
							sol->remove_a_bin(r_index, j - 1);  // After removing the first node, j-1 is the second node
							sol->remove_a_bin(r_index, k - 2);  // After removing the first two nodes, k-2 is the third node

							// Try reinserting the 3-block in different positions
							for (int insert_pos = start_point; insert_pos < end_point; insert_pos++)
							{
								// Insert the 3-block of nodes at the new position
								sol->insert_a_bin(r_index, insert_pos, node1);
								sol->insert_a_bin(r_index, insert_pos + 1, node2);
								sol->insert_a_bin(r_index, insert_pos + 2, node3);

								// Check if the new sequence improves the solution
								double new_obj = sol->objfunc;
								bool new_feasible = sol->isFeasible;

								// If improvement is found
								if (new_obj < sol->objfunc)
								{
									isImproved = true;
									sol->objfunc = new_obj; // Update the solution's objective function
									break;  // Stop further exploration for this block
								}
								else
								{
									// Revert the insertion if it did not improve
									sol->remove_a_bin(r_index, insert_pos);
									sol->remove_a_bin(r_index, insert_pos + 1);
									sol->remove_a_bin(r_index, insert_pos + 2);
								}
							}

							// Restore the original sequence if no improvement was made
							if (!isImproved)
							{
								sol->insert_a_bin(r_index, i, node1);
								sol->insert_a_bin(r_index, j, node2);
								sol->insert_a_bin(r_index, k, node3);
							}
						}
					}
				}
			}
			// Update start_point for the next trip
			start_point = end_point + 1;
		}
	}
	return isImproved;
}
*/
