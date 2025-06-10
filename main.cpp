#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// パラメータ
struct SimulationInfo {
  double LX, LY;
  const double alpha = 0.25;
  const double beta = 2.5;
  const double a = 0.5; // 感応度
  const double b = 1.0;
  const double c = -1.0;
  const double dt = 0.5; // 時間刻み
  const double R = 2.0;  // 隣接歩行者を探す距離
};

// 歩行者クラス
struct People {
  double x, y;
  double vx, vy;
  double fx, fy;
  double v0_x, v0_y;
  std::vector<int> neighbors;
};

void calculate_force(int j, int k, std::vector<People> &agents, SimulationInfo &si) {
  auto &pj = agents[j];
  const auto &pk = agents[k];

  // r_kj = x_k - x_j
  double dx = pk.x - pj.x;
  double dy = pk.y - pj.y;

  if (dx < -si.LX * 0.5) dx += si.LX;
  if (dx > si.LX * 0.5) dx -= si.LX;
  if (dy < -si.LY * 0.5) dy += si.LY;
  if (dy > si.LY * 0.5) dy -= si.LY;

  double r = std::sqrt(dx * dx + dy * dy);
  double v0_norm = std::sqrt(pj.v0_x * pj.v0_x + pj.v0_y * pj.v0_y);

  // cos(phi) = (r_kj ⋅ v0_j) / (|r_kj| * |v0_j|)
  double cos_phi = (dx * pj.v0_x + dy * pj.v0_y) / (r * v0_norm);
  // n_kj = r_kj / |r_kj|
  double n_x = dx / r;
  double n_y = dy / r;

  double force_magnitude = si.alpha * (std::tanh(si.beta * (r - si.b)) + si.c);
  // 全体の力積ベクトル: dt * f(r_kj) * (1 + cos_phi) * n_kj
  double fx = force_magnitude * (1.0 + cos_phi) * n_x;
  double fy = force_magnitude * (1.0 + cos_phi) * n_y;

  // 力の追加
  pj.fx += fx;
  pj.fy += fy;
}

// 隣接歩行者を探す
void update_neighbors(std::vector<People> &agents, SimulationInfo &si) {
  const int N = agents.size();
  const double R2 = si.R * si.R;

  for (auto &p : agents) {
    p.neighbors.clear();
  }

  for (int j = 0; j < N; ++j) {
    for (int k = j + 1; k < N; ++k) {
      double dx = agents[k].x - agents[j].x;
      double dy = agents[k].y - agents[j].y;
      if (dx < -si.LX * 0.5) dx += si.LX;
      if (dy < -si.LY * 0.5) dy += si.LY;
      if (dx > si.LX * 0.5) dx -= si.LX;
      if (dy > si.LY * 0.5) dy -= si.LY;

      double dist2 = dx * dx + dy * dy;

      if (dist2 <= R2) {
        agents[j].neighbors.push_back(k);
        agents[k].neighbors.push_back(j);
      }
    }
  }
}

void initialize_agents_triangular_lattice(std::vector<People> &agents,
                                          SimulationInfo &si,
                                          const double r,
                                          std::mt19937 &rng) {
  const int nx = 20;
  const int ny = 20;

  const double dx = r;
  const double dy = std::sqrt(3.0) / 2.0 * r;

  si.LX = nx * dx;
  si.LY = ny * dy;

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
  // GIFアニメーション用の元データ(frame????.dat)の出力
  static int index = 0;
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

  ++index;
}

void step_simulation(std::vector<People> &agents, SimulationInfo &si) {
  for (auto &p : agents) {
    p.fx = 0.0;
    p.fy = 0.0;
  }

  // 近傍リストの更新
  update_neighbors(agents, si);

  // jが受ける力を各kから計算して積算
  for (int j = 0; j < agents.size(); ++j) {
    for (int k : agents[j].neighbors) {
      calculate_force(j, k, agents, si);
    }
  }

  // 運動量、位置の更新
  for (auto &p : agents) {
    p.vx += si.a * (p.v0_x + p.fx - p.vx) * si.dt;
    p.vy += si.a * (p.v0_y + p.fy - p.vy) * si.dt;

    p.x += p.vx * si.dt;
    p.y += p.vy * si.dt;

    // 周期境界条件
    if (p.x < 0) p.x += si.LX;
    if (p.x >= si.LX) p.x -= si.LX;
    if (p.y < 0) p.y += si.LY;
    if (p.y >= si.LY) p.y -= si.LY;
  }
}

// Pythonにあとで渡すためのYAMLファイルを作成
void save_yaml(SimulationInfo &si, const std::string &filename = "conf.yaml") {
  std::ofstream ofs(filename);
  if (!ofs) {
    std::cerr << "Error: Failed to open " << filename << " for writing." << std::endl;
    return;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "LX: " << si.LX << "\n";
  ofs << "LY: " << si.LY << "\n";
}

void simulation() {
  SimulationInfo si;
  const int seed = 12346;
  std::mt19937 rng(seed);
  std::vector<People> agents;

  std::stringstream ss_stgraph; // 時空図用

  const double r = 1.3;
  initialize_agents_triangular_lattice(agents, si, r, rng);
  std::cout << "# " << si.LX << " " << si.LY << agents.size() << std::endl;
  write_agent_frame(agents);
  for (int i = 0; i < 400; i++) {
    step_simulation(agents, si);
    write_agent_frame(agents);
    for (auto &a : agents) {
      ss_stgraph << i * si.dt << " " << a.x << std::endl;
    }
  }
  save_yaml(si);

  // 時空図の保存
  std::ofstream ofs("spatio_temporal.dat");
  if (!ofs) {
    std::cerr << "Error: Failed to open spatio_temporal.dat for writing." << std::endl;
  } else {
    ofs << ss_stgraph.str();
  }
}

int main() {
  simulation();
}