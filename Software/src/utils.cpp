#include <iostream>
#include <vector>
#include "utils.h"

std::vector<int> multiply(const std::vector<std::vector<int>>& matrix, 
                          const std::vector<int>& vector) {
  int rows = matrix.size();
  int cols = matrix[0].size();

  std::vector<int> result(rows);
  for (int i = 0; i < rows; i++) {
    int sum = 0;
    for (int j = 0; j < cols; j++) {
      sum += matrix[i][j] * vector[j];
    }
    result[i] = sum;
  }

  return result;
}

/*
int test() {
  std::vector<std::vector<int>> matrix = {{1, 2, 3}, 
                                          {4, 5, 6}, 
                                          {7, 8, 9}};
  std::vector<int> vector = {1, 2, 3};
  std::vector<int> result = multiply(matrix, vector);

  for (const auto& item : result) {
    std::cout << item << " ";
  }

  return 0;
}
*/

