// Dashboard state model for the RCA TUI.
//
// The TUI is a visualisation layer only: it parses the JSON published by the
// Python diagnostic monitor on /rca/dashboard_state and renders it. No RCA
// logic lives here. Everything in this header is plain C++17 with no ROS or
// FTXUI dependency so that it can be unit-tested in isolation.
#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace diagnostic_tui {

constexpr int kSupportedSchemaVersion = 1;

struct NodeInfo {
  std::string name;
  std::string status;          // NORMAL | ANOMALOUS | ROOT_CAUSE | MISSING
  int anomaly_count = 0;
  double max_severity = 0.0;
  std::string last_metric;
  // ROS/DDS graph membership as of the monitor's last discovery sweep. A crashed
  // process stays in the graph until its DDS participant lease expires, so this
  // is NOT an instantaneous crash indicator.
  bool in_ros_graph = true;
  // Runtime liveness derived from observed data flow: ALIVE | STARVED | UNKNOWN.
  // This is what detects a dead node quickly; graph membership confirms it later.
  std::string liveness = "UNKNOWN";
};

struct EdgeInfo {
  std::string from;
  std::string to;
  std::vector<std::string> topics;
};

struct AnomalyInfo {
  std::string node;
  std::string type;
  double timestamp = 0.0;
  std::optional<double> onset_time;
  double severity = 0.0;
  double value = 0.0;
  std::string description;
  bool informational = false;
};

struct CandidateInfo {
  std::string node;
  double total = 0.0, d = 0.0, t = 0.0, s = 0.0, a = 0.0;
};

struct DiagnosisInfo {
  double timestamp = 0.0;
  std::string root_cause;
  double confidence = 0.0;
  double dependency = 0.0, temporal = 0.0, symptom = 0.0, severity = 0.0;
  std::map<std::string, double> weights;
  std::vector<CandidateInfo> candidates;
  std::vector<std::string> propagation_chain;
  std::vector<std::string> anomalous_nodes;
  std::string explanation;
};

struct TimelineEvent {
  double t = 0.0;
  std::string node;
  std::string event;
  double severity = 0.0;
  std::string message;
};

struct Counts {
  int nodes = 0, edges = 0, topics = 0, anomalies_in_window = 0, anomalous_nodes = 0;
  int diagnoses_total = 0, anomalies_total = 0;
};

struct DashboardState {
  int schema_version = 0;
  double timestamp = 0.0;          // monitor time when the state was built
  // Backend that produced the data ("rca_sim", "gazebo", ...). Presentation
  // metadata published by the monitor; never evaluation ground truth.
  std::string simulator = "unknown";
  double monitor_uptime_sec = 0.0;
  std::string system_status;       // WAITING | NORMAL | DEGRADED | RECOVERING
  Counts counts;
  double rca_window_sec = 0.0;
  std::vector<NodeInfo> nodes;
  std::vector<EdgeInfo> edges;
  std::vector<AnomalyInfo> anomalies;
  std::optional<DiagnosisInfo> latest_diagnosis;
  std::vector<TimelineEvent> timeline;
};

struct ParseResult {
  bool ok = false;
  std::string error;               // human readable reason when !ok
};

// Parses one /rca/dashboard_state payload. Missing optional fields get defaults;
// malformed JSON or an unsupported schema_version yields ok=false and leaves
// `out` untouched.
ParseResult parse_dashboard_json(const std::string& json_text, DashboardState& out);

// ---------------------------------------------------------------- shared state

enum class ConnectionStatus { kWaiting, kLive, kStale, kError };

struct Snapshot {
  bool has_state = false;
  DashboardState state;
  ConnectionStatus connection = ConnectionStatus::kWaiting;
  double age_sec = 0.0;            // wall-clock seconds since the last *valid* message
  std::string last_error;          // last parse error (kept for display, cleared on valid)
  std::uint64_t messages_ok = 0;
  std::uint64_t messages_rejected = 0;
};

// Thread-safe holder written by the ROS thread and read by the UI thread.
class SharedState {
 public:
  explicit SharedState(double stale_after_sec = 3.0) : stale_after_sec_(stale_after_sec) {}

  // Returns true when the payload was accepted. `now_sec` is the receiver's clock.
  bool update_from_json(const std::string& json_text, double now_sec);
  Snapshot snapshot(double now_sec) const;
  double stale_after_sec() const { return stale_after_sec_; }

 private:
  mutable std::mutex mutex_;
  double stale_after_sec_;
  bool has_state_ = false;
  DashboardState state_;
  double last_ok_sec_ = 0.0;
  std::string last_error_;
  std::uint64_t ok_ = 0, rejected_ = 0;
};

// ------------------------------------------------------------- view models

// One row of the DAG layout: nodes at the same topological depth.
struct GraphLayer {
  std::vector<NodeInfo> nodes;
};

struct GraphView {
  std::vector<GraphLayer> layers;              // depth 0 = sources (no incoming edges)
  std::vector<EdgeInfo> edges;                 // as received (already dynamic)
  std::vector<std::string> cycle_nodes;        // nodes that could not be layered (cycle)
};

// Longest-path layering of the received DAG. Purely derived from nodes/edges in
// the state; nothing about the demo topology is assumed.
GraphView layout_graph(const DashboardState& state);

struct DiagnosisView {
  bool present = false;
  std::string root_cause = "None";
  std::string confidence = "-";
  std::vector<std::pair<std::string, double>> scores;   // label -> value (0..1)
  std::string chain;                                    // "a -> b -> c"
  std::vector<std::string> evidence_lines;              // explanation text as lines
  std::vector<CandidateInfo> candidates;
};

DiagnosisView diagnosis_view(const DashboardState& state);

// Formats a unix timestamp as local HH:MM:SS.cc for the timeline.
std::string format_clock(double unix_sec);

// Human readable status label for a node ("[!]", "[X]", "[*]", "[ ]").
std::string node_marker(const std::string& status);

}  // namespace diagnostic_tui
