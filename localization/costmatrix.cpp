#include <vector>
#include <algorithm>
using namespace std;
// Function to create a cost matrix using two vectors
vector<vector<double>> makeHungarianMatrix(const vector<double>& vec1, const vector<double>& vec2) {
    // Get the size of the vectors
    int n = vec1.size();
    int m = vec2.size();

    // Create the cost matrix
    vector<vector<double>> costMatrix(n, vector<double>(m));

    // Fill the cost matrix with the absolute difference between the elements of the two vectors
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            costMatrix[i][j] = abs(vec1[i] - vec2[j]);
        }
    }

    return costMatrix;
}