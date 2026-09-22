#include "diagnostic_tui/dashboard_state.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>

#include <nlohmann/json.hpp>

namespace diagnostic_tui {

using json = nlohmann::json;

namespace {

template <typename T>
T get_or(const json& j, const char* key, const T& fallback) {
  if (!j.is_object()) return fallback;
  auto it = j.find(key);
  if (it == j.end() || it->is_null()) return fallback;
  try {
    return it->get<T>();
  } catch (const json::exception&) {
    return fallback;
  }
}

std::vector<std::string> string_list(const json& j, const char* key) {
  std::vector<std::string> out;
  if (!j.is_object()) return out;
  auto it = j.find(key);
  if (it == j.end() || !it->is_array()) return out;
  for (const auto& v : *it) {
    if (v.is_string()) out.push_back(v.get<std::string>());
  }
  return out;
}

}  // namespace

ParseResult parse_dashboard_json(const std::string& json_text, DashboardState& out) {
  json j;
  try {
    j = json::parse(json_text);
  } catch (const json::exception& e) {
    return {false, std::string("invalid JSON: ") + e.what()};
  }
  if (!j.is_object()) return {false, "payload is not a JSON object"};

  const int version = get_or<int>(j, "schema_version", -1);
  if (version != kSupportedSchemaVersion) {
    return {false, "unsupported schema_version " + std::to_string(version) +
                       " (expected " + std::to_string(kSupportedSchemaVersion) + ")"};
  }

  DashboardState s;
  s.schema_version = version;
  s.timestamp = get_or<double>(j, "timestamp", 0.0);
  s.simulator = get_or<std::string>(j, "simulator", "unknown");
  s.monitor_uptime_sec = get_or<double>(j, "monitor_uptime_sec", 0.0);
  s.system_status = get_or<std::string>(j, "system_status", "UNKNOWN");
  s.rca_window_sec = get_or<double>(j, "rca_window_sec", 0.0);

  if (j.contains("counts") && j["counts"].is_object()) {
    const auto& c = j["counts"];
    s.counts.nodes = get_or<int>(c, "nodes", 0);
    s.counts.edges = get_or<int>(c, "edges", 0);
    s.counts.topics = get_or<int>(c, "topics", 0);
    s.counts.anomalies_in_window = get_or<int>(c, "anomalies_in_window", 0);
    s.counts.anomalous_nodes = get_or<int>(c, "anomalous_nodes", 0);
    s.counts.diagnoses_total = get_or<int>(c, "diagnoses_total", 0);
    s.counts.anomalies_total = get_or<int>(c, "anomalies_total", 0);
  }

  if (j.contains("nodes") && j["nodes"].is_array()) {
    for (const auto& n : j["nodes"]) {
      if (!n.is_object()) continue;
      NodeInfo ni;
      ni.name = get_or<std::string>(n, "name", "");
      if (ni.name.empty()) continue;
      ni.status = get_or<std::string>(n, "status", "NORMAL");
      ni.anomaly_count = get_or<int>(n, "anomaly_count", 0);
      ni.max_severity = get_or<double>(n, "max_severity", 0.0);
      ni.last_metric = get_or<std::string>(n, "last_metric", "");
      ni.in_ros_graph = get_or<bool>(n, "in_ros_graph", true);
      ni.liveness = get_or<std::string>(n, "liveness", "UNKNOWN");
      s.nodes.push_back(ni);
    }
  }
  if (s.counts.nodes == 0) s.counts.nodes = static_cast<int>(s.nodes.size());

  if (j.contains("edges") && j["edges"].is_array()) {
    for (const auto& e : j["edges"]) {
      if (!e.is_object()) continue;
      EdgeInfo ei;
      ei.from = get_or<std::string>(e, "from", "");
      ei.to = get_or<std::string>(e, "to", "");
      if (ei.from.empty() || ei.to.empty()) continue;
      ei.topics = string_list(e, "topics");
      s.edges.push_back(ei);
    }
  }
  if (s.counts.edges == 0) s.counts.edges = static_cast<int>(s.edges.size());

  if (j.contains("anomalies") && j["anomalies"].is_array()) {
    for (const auto& a : j["anomalies"]) {
      if (!a.is_object()) continue;
      AnomalyInfo ai;
      ai.node = get_or<std::string>(a, "node", "");
      ai.type = get_or<std::string>(a, "type", "");
      ai.timestamp = get_or<double>(a, "timestamp", 0.0);
      if (a.contains("onset_time") && a["onset_time"].is_number()) ai.onset_time = a["onset_time"].get<double>();
      ai.severity = get_or<double>(a, "severity", 0.0);
      ai.value = get_or<double>(a, "value", 0.0);
      ai.description = get_or<std::string>(a, "description", "");
      ai.informational = get_or<bool>(a, "informational", false);
      s.anomalies.push_back(ai);
    }
  }

  if (j.contains("latest_diagnosis") && j["latest_diagnosis"].is_object()) {
    const auto& d = j["latest_diagnosis"];
    DiagnosisInfo di;
    di.timestamp = get_or<double>(d, "timestamp", 0.0);
    di.root_cause = get_or<std::string>(d, "root_cause", "");
    di.confidence = get_or<double>(d, "confidence", 0.0);
    if (d.contains("scores") && d["scores"].is_object()) {
      const auto& sc = d["scores"];
      di.dependency = get_or<double>(sc, "dependency", 0.0);
      di.temporal = get_or<double>(sc, "temporal", 0.0);
      di.symptom = get_or<double>(sc, "symptom", 0.0);
      di.severity = get_or<double>(sc, "severity", 0.0);
    }
    if (d.contains("weights") && d["weights"].is_object()) {
      for (auto it = d["weights"].begin(); it != d["weights"].end(); ++it) {
        if (it->is_number()) di.weights[it.key()] = it->get<double>();
      }
    }
    if (d.contains("candidates") && d["candidates"].is_array()) {
      for (const auto& c : d["candidates"]) {
        if (!c.is_object()) continue;
        CandidateInfo ci;
        ci.node = get_or<std::string>(c, "node", "");
        ci.total = get_or<double>(c, "total", 0.0);
        ci.d = get_or<double>(c, "d", 0.0);
        ci.t = get_or<double>(c, "t", 0.0);
        ci.s = get_or<double>(c, "s", 0.0);
        ci.a = get_or<double>(c, "a", 0.0);
        if (!ci.node.empty()) di.candidates.push_back(ci);
      }
    }
    di.propagation_chain = string_list(d, "propagation_chain");
    di.anomalous_nodes = string_list(d, "anomalous_nodes");
    di.explanation = get_or<std::string>(d, "explanation", "");
    if (!di.root_cause.empty()) s.latest_diagnosis = di;
  }

  if (j.contains("timeline") && j["timeline"].is_array()) {
    for (const auto& e : j["timeline"]) {
      if (!e.is_object()) continue;
      TimelineEvent te;
      te.t = get_or<double>(e, "t", 0.0);
      te.node = get_or<std::string>(e, "node", "");
      te.event = get_or<std::string>(e, "event", "");
      te.severity = get_or<double>(e, "severity", 0.0);
      te.message = get_or<std::string>(e, "message", "");
      s.timeline.push_back(te);
    }
  }

  out = std::move(s);
  return {true, ""};
}

// ---------------------------------------------------------------- SharedState

bool SharedState::update_from_json(const std::string& json_text, double now_sec) {
  DashboardState parsed;
  const ParseResult r = parse_dashboard_json(json_text, parsed);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!r.ok) {
    ++rejected_;
    last_error_ = r.error;
    return false;
  }
  state_ = std::move(parsed);
  has_state_ = true;
  last_ok_sec_ = now_sec;
  last_error_.clear();
  ++ok_;
  return true;
}

Snapshot SharedState::snapshot(double now_sec) const {
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot snap;
  snap.has_state = has_state_;
  snap.state = state_;
  snap.messages_ok = ok_;
  snap.messages_rejected = rejected_;
  snap.last_error = last_error_;
  if (!has_state_) {
    snap.connection = last_error_.empty() ? ConnectionStatus::kWaiting : ConnectionStatus::kError;
    snap.age_sec = 0.0;
  } else {
    snap.age_sec = std::max(0.0, now_sec - last_ok_sec_);
    snap.connection = snap.age_sec > stale_after_sec_ ? ConnectionStatus::kStale : ConnectionStatus::kLive;
  }
  return snap;
}

// ---------------------------------------------------------------- view models

GraphView layout_graph(const DashboardState& state) {
  GraphView view;
  view.edges = state.edges;

  std::map<std::string, NodeInfo> by_name;
  for (const auto& n : state.nodes) by_name[n.name] = n;
  // Nodes referenced only by edges still get a (status-less) box
  for (const auto& e : state.edges) {
    for (const auto& name : {e.from, e.to}) {
      if (!by_name.count(name)) {
        NodeInfo ni;
        ni.name = name;
        ni.status = "UNKNOWN";
        by_name[name] = ni;
      }
    }
  }
  if (by_name.empty()) return view;

  // Longest-path layering via Kahn's algorithm (deterministic: names sorted).
  std::map<std::string, int> indegree;
  std::map<std::string, std::vector<std::string>> succ;
  for (const auto& kv : by_name) indegree[kv.first] = 0;
  std::set<std::pair<std::string, std::string>> seen_edges;
  for (const auto& e : state.edges) {
    if (e.from == e.to || !seen_edges.insert({e.from, e.to}).second) continue;
    succ[e.from].push_back(e.to);
    indegree[e.to]++;
  }
  std::map<std::string, int> depth;
  std::vector<std::string> ready;
  for (const auto& kv : indegree) {
    if (kv.second == 0) {
      ready.push_back(kv.first);
      depth[kv.first] = 0;
    }
  }
  std::set<std::string> placed;
  while (!ready.empty()) {
    std::sort(ready.begin(), ready.end());
    const std::string u = ready.front();
    ready.erase(ready.begin());
    placed.insert(u);
    for (const auto& v : succ[u]) {
      depth[v] = std::max(depth.count(v) ? depth[v] : 0, depth[u] + 1);
      if (--indegree[v] == 0) ready.push_back(v);
    }
  }
  int max_depth = 0;
  for (const auto& n : placed) max_depth = std::max(max_depth, depth[n]);
  if (!placed.empty()) view.layers.resize(static_cast<std::size_t>(max_depth) + 1);
  for (const auto& kv : by_name) {
    if (!placed.count(kv.first)) {
      view.cycle_nodes.push_back(kv.first);   // part of a cycle: cannot be layered
      continue;
    }
    view.layers[static_cast<std::size_t>(depth[kv.first])].nodes.push_back(kv.second);
  }
  // Cycle members are shown in an extra trailing layer so nothing disappears
  if (!view.cycle_nodes.empty()) {
    GraphLayer extra;
    for (const auto& n : view.cycle_nodes) extra.nodes.push_back(by_name[n]);
    view.layers.push_back(extra);
  }
  return view;
}

DiagnosisView diagnosis_view(const DashboardState& state) {
  DiagnosisView v;
  if (!state.latest_diagnosis) return v;
  const auto& d = *state.latest_diagnosis;
  v.present = true;
  v.root_cause = d.root_cause;
  std::ostringstream conf;
  conf << std::fixed << std::setprecision(3) << d.confidence;
  v.confidence = conf.str();
  v.scores = {{"Dependency", d.dependency}, {"Temporal", d.temporal},
              {"Symptoms", d.symptom}, {"Severity", d.severity}};
  for (std::size_t i = 0; i < d.propagation_chain.size(); ++i) {
    if (i) v.chain += " -> ";
    v.chain += d.propagation_chain[i];
  }
  std::istringstream in(d.explanation);
  std::string line;
  while (std::getline(in, line)) {
    // Drop pure separator lines; keep the evidence text exactly as produced.
    if (line.find_first_not_of("= ") == std::string::npos) continue;
    v.evidence_lines.push_back(line);
  }
  v.candidates = d.candidates;
  return v;
}

std::string format_clock(double unix_sec) {
  if (unix_sec <= 0.0) return "--:--:--.--";
  const std::time_t secs = static_cast<std::time_t>(std::floor(unix_sec));
  const int centis = static_cast<int>(std::floor((unix_sec - static_cast<double>(secs)) * 100.0));
  std::tm tm_buf{};
  localtime_r(&secs, &tm_buf);
  std::ostringstream out;
  out << std::put_time(&tm_buf, "%H:%M:%S") << '.' << std::setw(2) << std::setfill('0') << centis;
  return out.str();
}

std::string node_marker(const std::string& status) {
  if (status == "ROOT_CAUSE") return "[*]";
  if (status == "ANOMALOUS") return "[!]";
  if (status == "MISSING") return "[X]";
  if (status == "NORMAL") return "[ ]";
  return "[?]";
}

}  // namespace diagnostic_tui
