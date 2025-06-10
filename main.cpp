#include <cmath>
#include <iostream>
#include <vector>

// パラメータ

const double alpha = 0.25;
const double beta = 2.5;
const double b = 1.0;
const double c = -1.0;
const double dt = 0.01;

struct People {
  double x, y;
  double vx, vy;
  double v0_x, v0_y;
};

void calculate_force(int j, int k, std::vector<People> &agents) {
  auto &pj = agents[j];
  const auto &pk = agents[k];

  // r_kj = x_k - x_j
  double dx = pk.x - pj.x;
  double dy = pk.y - pj.y;

  double r = std::sqrt(dx * dx + dy * dy);
  double v0_norm = std::sqrt(pj.v0_x * pj.v0_x + pj.v0_y * pj.v0_y);

  // cos(phi) = (r_kj ⋅ v0_j) / (|r_kj| * |v0_j|)
  double cos_phi = (dx * pj.v0_x + dy * pj.v0_y) / (r * v0_norm);
  // n_kj = r_kj / |r_kj|
  double n_x = dx / r;
  double n_y = dy / r;

  double force_magnitude = alpha * (std::tanh(beta * (r - b)) + c);
  // 全体の力積ベクトル: dt * f(r_kj) * (1 + cos_phi) * n_kj
  double fx = dt * force_magnitude * (1.0 + cos_phi) * n_x;
  double fy = dt * force_magnitude * (1.0 + cos_phi) * n_y;

  // 速度の更新
  pj.vx += fx;
  pj.vy += fy;
}

int main() {
}