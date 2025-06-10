#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
// パラメータ

const double alpha = 0.25;
const double beta = 2.5;
const double a = 0.5;
const double b = 1.0;
const double c = -1.0;
const double dt = 0.5;
const double R = 2.0; // Search Length

// システムサイズ

double LX, LY;

struct People {
  double x, y;
  double vx, vy;
  double fx, fy;
  double v0_x, v0_y;
  std::vector<int> neighbors;
};

void calculate_force(int j, int k, std::vector<People> &agents) {
  auto &pj = agents[j];
  const auto &pk = agents[k];

  // r_kj = x_k - x_j
  double dx = pk.x - pj.x;
  double dy = pk.y - pj.y;

  if (dx < -LX * 0.5) dx += LX;
  if (dx > LX * 0.5) dx -= LX;
  if (dy < -LY * 0.5) dy += LY;
  if (dy > LY * 0.5) dy -= LY;

  double r = std::sqrt(dx * dx + dy * dy);
  double v0_norm = std::sqrt(pj.v0_x * pj.v0_x + pj.v0_y * pj.v0_y);

  // cos(phi) = (r_kj ⋅ v0_j) / (|r_kj| * |v0_j|)
  double cos_phi = (dx * pj.v0_x + dy * pj.v0_y) / (r * v0_norm);
  // n_kj = r_kj / |r_kj|
  double n_x = dx / r;
  double n_y = dy / r;

  double force_magnitude = alpha * (std::tanh(beta * (r - b)) + c);
  // 全体の力積ベクトル: dt * f(r_kj) * (1 + cos_phi) * n_kj
  double fx = force_magnitude * (1.0 + cos_phi) * n_x;
  double fy = force_magnitude * (1.0 + cos_phi) * n_y;

  // 力の追加
  pj.fx += fx;
  pj.fy += fy;
}

void update_neighbors(std::vector<People> &agents, double R) {
  const int N = agents.size();
  const double R2 = R * R;

  for (auto &p : agents) {
    p.neighbors.clear();
  }

  for (int j = 0; j < N; ++j) {
    for (int k = j + 1; k < N; ++k) {
      double dx = agents[k].x - agents[j].x;
      double dy = agents[k].y - agents[j].y;
      if (dx < -LX * 0.5) dx += LX;
      if (dy < -LY * 0.5) dy += LY;
      if (dx > LX * 0.5) dx -= LX;
      if (dy > LY * 0.5) dy -= LY;

      double dist2 = dx * dx + dy * dy;

      if (dist2 <= R2) {
        agents[j].neighbors.push_back(k);
        agents[k].neighbors.push_back(j);
      }
    }
  }
}

void initialize_agents_triangular_lattice(std::vector<People> &agents,
                                          double &LX, double &LY,
                                          const double r,
                                          std::mt19937 &rng) {
  const int nx = 20;
  const int ny = 20;

  const double dx = r;
  const double dy = std::sqrt(3.0) / 2.0 * r;

  LX = nx * dx;
  LY = ny * dy;

  const int N = nx * ny;
  agents.clear();
  agents.reserve(N);

  std::uniform_real_distribution<double> jitter(-0.01, 0.01); // rに対する割合で揺らぎ

  for (int iy = 0; iy < ny; ++iy) {
    for (int ix = 0; ix < nx; ++ix) {
      People p;
      double x = ix * dx;
      double y = iy * dy;

      if (iy % 2 == 1) {
        x += dx / 2.0; // 奇数行はオフセット
      }

      x += jitter(rng);
      y += jitter(rng);

      p.x = x;
      p.y = y;
      p.vx = 1.0;
      p.vy = 0.0;
      p.v0_x = 1.0;
      p.v0_y = 0.0;

      agents.push_back(p);
    }
  }
}

void write_agent_frame(const std::vector<People> &agents) {
  static int index = 0;

  // ファイル名を作成：frame0000.dat の形式
  std::ostringstream filename;
  filename << "frame" << std::setfill('0') << std::setw(4) << index << ".dat";

  std::ofstream ofs(filename.str());
  std::cout << "# " << filename.str() << std::endl;
  if (!ofs) {
    std::cerr << "Error: Failed to open file " << filename.str() << " for writing." << std::endl;
    return;
  }

  for (const auto &p : agents) {
    ofs << p.x << " " << p.y << " " << p.vx << " " << p.vy << "\n";
  }

  ++index; // 次回呼び出し時のインデックスを更新
}

void step_simulation(std::vector<People> &agents) {

  for (auto &p : agents) {
    p.fx = 0.0;
    p.fy = 0.0;
  }

  // 1. 近傍リストの更新
  update_neighbors(agents, R);

  // 2. 力積による速度更新（全対：jが受ける力を各kから計算）
  for (int j = 0; j < agents.size(); ++j) {
    for (int k : agents[j].neighbors) {
      calculate_force(j, k, agents);
    }
  }

  // 3. 位置更新 ＆ 周期境界処理
  for (auto &p : agents) {
    p.vx += a * (p.v0_x + p.fx - p.vx) * dt;
    p.vy += a * (p.v0_y + p.fy - p.vy) * dt;

    p.x += p.vx * dt;
    p.y += p.vy * dt;

    // 周期境界条件
    if (p.x < 0) p.x += LX;
    if (p.x >= LX) p.x -= LX;
    if (p.y < 0) p.y += LY;
    if (p.y >= LY) p.y -= LY;
  }
}

void save_yaml(const std::string &filename = "conf.yaml") {
  std::ofstream ofs(filename);
  if (!ofs) {
    std::cerr << "Error: Failed to open " << filename << " for writing." << std::endl;
    return;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "LX: " << LX << "\n";
  ofs << "LY: " << LY << "\n";
}

void test() {
  const int seed = 12346;
  std::mt19937 rng(seed);
  std::vector<People> agents;
  const double r = 1.3;
  initialize_agents_triangular_lattice(agents, LX, LY, r, rng);
  const double R = 1.1;
  update_neighbors(agents, R);
}

void simulation() {
  const int seed = 12346;
  std::mt19937 rng(seed);
  std::vector<People> agents;
  const double r = 1.3;
  initialize_agents_triangular_lattice(agents, LX, LY, r, rng);
  std::cout << "# " << LX << " " << LY << agents.size() << std::endl;
  write_agent_frame(agents);
  for (int i = 0; i < 400; i++) {
    step_simulation(agents);
    write_agent_frame(agents);
    for (auto &a : agents) {
      printf("%f %f\n", i * dt, a.x);
    }
  }
  save_yaml();
}

int main() {
  simulation();
}