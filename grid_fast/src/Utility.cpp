#include "Utility.hh"

bool operator==(const point_int &lhs, const point_int &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}
bool operator!=(const point_int &lhs, const point_int &rhs) {
  return lhs.x != rhs.x || lhs.y != rhs.y;
}

double dist(point_int p1, point_int p2) {
  point_int d = {p1.x - p2.x, p1.y - p2.y};
  return sqrt(d.x * d.x + d.y * d.y);
}

double dist(point p1, point p2) {
  point d = {p1.x - p2.x, p1.y - p2.y};
  return sqrt(d.x * d.x + d.y * d.y);
}

double angleBetweenLines(point_int A, point_int B, point_int P, point_int Q) {
  double dot = (A.x - B.x) * (P.x - Q.x) + (A.y - B.y) * (P.y - Q.y);
  double det = dist(A, B) + dist(P, Q);
  return acos(det / dot);
}

// suport function for ant_step rotates the dir
point_int rotate_dir(point_int dir, bool cw) {
  point_int newDir[] = {{-1, 1}, {0, 1},   {1, 1},  {1, 0}, {1, -1},
                        {0, -1}, {-1, -1}, {-1, 0}, {0, 0}};
  for (int i = 0; i < 8; i++) {
    if (newDir[i] == dir) {
      int next_index = cw ? i + 1 : i - 1;
      if (next_index == -1)
        next_index = 7;
      next_index = next_index % 8;
      return newDir[next_index];
    }
  }
  return newDir[8];
}

vector<point_int> drawLine(point_int start, point_int end) {
  vector<point_int> p;
  point_int lenght = {end.x - start.x, end.y - start.y};
  point_int step = {lenght.x / std::abs(lenght.x),
                    lenght.y / std::abs(lenght.y)};
  lenght = {std::abs(lenght.x), std::abs(lenght.y)};
  point_int steps_taken = {0, 0};

  p.push_back(start);

  for (int s = 0; s < lenght.x + lenght.y; s++) {
    if (steps_taken.x * lenght.y < steps_taken.y * lenght.x && lenght.x != 0 ||
        lenght.y == 0) {
      steps_taken.x += 1;
      start.x += step.x;
    } else {
      steps_taken.y += 1;
      start.y += step.y;
    }
    p.push_back(start);
  }
  return p;
}

void print(string s) {
  // ROS_INFO("%s",s.c_str());
}

vector<point_int> fillPoly(vector<point_int> PL) {
  point_int maxP = {-1, -1}, minP = {-1, -1};
  vector<vector<bool>> newMap;
  for (int n = 0; n < PL.size(); n++) {
    if (PL[n].x > maxP.x)
      maxP.x = PL[n].x;
    if (PL[n].y > maxP.y)
      maxP.y = PL[n].y;
    if (PL[n].x < minP.x || minP.x == -1)
      minP.x = PL[n].x;
    if (PL[n].y < minP.y || minP.y == -1)
      minP.y = PL[n].y;
  }
  newMap.resize(maxP.x - minP.x + 1);
  for (int x = 0; x < newMap.size(); x++) {
    newMap[x].resize(maxP.y - minP.y + 1);
    for (int y = 0; y < newMap[x].size(); y++)
      newMap[x][y] = false;
  }
  for (int n = 0; n < PL.size(); n++) {
    int dy = PL[(n + 1) % PL.size()].y - PL[n].y;
    if (dy == 0)
      continue;
    int y = dy == 1 ? PL[n].y : PL[n].y - 1;
    y -= minP.y;
    if (y < 0)
      continue;
    for (int x = 0; x <= PL[n].x - minP.x; x++) {
      newMap[x][y] = !newMap[x][y];
    }
  }
  vector<point_int> filledPoints;
  for (int x = 0; x < newMap.size(); x++) {
    for (int y = 0; y < newMap[x].size(); y++) {
      if (newMap[x][y]) {
        point_int p = {x + minP.x, y + minP.y + 1};
        filledPoints.push_back(p);
      }
    }
  }
  return filledPoints;
}

vector<point> fillPoints(point start, point end, double spacing,
                         bool lastAndFirst) {
  vector<point> out;
  point norm;
  double d = dist(start, end);
  norm.x = (end.x - start.x) / d * spacing;
  norm.y = (end.y - start.y) / d * spacing;

  for (int s = lastAndFirst ? 0 : 1; s < (int)(d / spacing); s++) {
    point p;
    p.x = start.x + norm.x * s;
    p.y = start.y + norm.y * s;
    out.push_back(p);
  }
  if (lastAndFirst) {
    out.push_back(end);
  }

  return out;
}

vector<point_int> fillPoints(point_int start, point_int end, double spacing,
                             bool lastAndFirst) {
  vector<point_int> out;
  point norm;
  double d = dist(start, end);
  norm.x = (end.x - start.x) / d * spacing;
  norm.y = (end.y - start.y) / d * spacing;

  for (int s = lastAndFirst ? 0 : 1; s < (int)(d / spacing); s++) {
    point_int p;
    p.x = start.x + std::round(norm.x * s);
    p.y = start.y + std::round(norm.y * s);
    out.push_back(p);
  }
  if (lastAndFirst) {
    out.push_back(end);
  }

  return out;
}
