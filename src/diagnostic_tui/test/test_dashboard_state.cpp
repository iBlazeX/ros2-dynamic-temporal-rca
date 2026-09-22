// Unit tests for the ROS-free dashboard model, view transforms and rendering.
#include <gtest/gtest.h>

#include <ftxui/screen/screen.hpp>
#include <string>

#include "diagnostic_tui/dashboard_state.hpp"
#include "diagnostic_tui/render.hpp"

using namespace diagnostic_tui;

namespace {

const char* kFullState = R"JSON({
  "schema_version": 1, "timestamp": 1000.0, "monitor_uptime_sec": 42.5, "system_status": "DEGRADED",
  "counts": {"nodes": 4, "edges": 3, "topics": 4, "anomalies_in_window": 5, "anomalous_nodes": 2,
             "diagnoses_total": 7, "anomalies_total": 9},
  "rca_window_sec": 6.0,
  "nodes": [
    {"name": "sensor_node", "status": "ROOT_CAUSE", "anomaly_count": 3, "max_severity": 1.0, "last_metric": "latency"},
    {"name": "perception_node", "status": "ANOMALOUS", "anomaly_count": 2, "max_severity": 0.9, "last_metric": "latency"},
    {"name": "localization_node", "status": "NORMAL"},
    {"name": "navigation_node", "status": "NORMAL"}
  ],
  "edges": [
    {"from": "sensor_node", "to": "perception_node", "topics": ["/sensor/scan"]},
    {"from": "perception_node", "to": "localization_node", "topics": ["/perception/obstacles"]},
    {"from": "localization_node", "to": "navigation_node", "topics": ["/localization/pose"]}
  ],
  "anomalies": [
    {"node": "sensor_node", "type": "latency", "timestamp": 999.9, "onset_time": 999.5, "severity": 1.0,
     "value": 0.4, "description": "late", "informational": false},
    {"node": "sensor_node", "type": "out_of_order", "timestamp": 999.95, "onset_time": null, "severity": 0.4,
     "value": -0.1, "description": "reordered", "informational": true}
  ],
  "latest_diagnosis": {
    "timestamp": 1000.0, "root_cause": "sensor_node", "confidence": 0.925,
    "scores": {"dependency": 1.0, "temporal": 1.0, "symptom": 1.0, "severity": 0.5},
    "weights": {"wD": 0.3, "wT": 0.3, "wS": 0.25, "wA": 0.15},
    "candidates": [{"node": "sensor_node", "total": 0.925, "d": 1, "t": 1, "s": 1, "a": 0.5},
                   {"node": "perception_node", "total": 0.7, "d": 0.5, "t": 0.9, "s": 0.5, "a": 0.9}],
    "propagation_chain": ["sensor_node", "perception_node"],
    "anomalous_nodes": ["perception_node", "sensor_node"],
    "explanation": "============\nROOT CAUSE DIAGNOSIS: sensor_node (Confidence: 0.925)\n============\nScore Breakdown\n  - Dependency D: 1.00"
  },
  "timeline": [
    {"t": 999.5, "node": "sensor_node", "event": "latency", "severity": 1.0, "message": "late"},
    {"t": 999.6, "node": "perception_node", "event": "latency", "severity": 0.9, "message": "late"},
    {"t": 1000.0, "node": "sensor_node", "event": "diagnosis", "severity": 0.925, "message": "probable root cause"}
  ]
})JSON";

std::string render_to_text(const ftxui::Element& e, int w = 160, int h = 50) {
  auto screen = ftxui::Screen::Create(ftxui::Dimension::Fixed(w), ftxui::Dimension::Fixed(h));
  ftxui::Render(screen, e);
  return screen.ToString();
}

}  // namespace

TEST(Parse, FullDocument) {
  DashboardState s;
  const auto r = parse_dashboard_json(kFullState, s);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(s.schema_version, 1);
  EXPECT_EQ(s.system_status, "DEGRADED");
  EXPECT_EQ(s.counts.nodes, 4);
  EXPECT_EQ(s.counts.anomalies_in_window, 5);
  ASSERT_EQ(s.nodes.size(), 4u);
  EXPECT_EQ(s.nodes[0].status, "ROOT_CAUSE");
  EXPECT_EQ(s.nodes[0].anomaly_count, 3);
  ASSERT_EQ(s.edges.size(), 3u);
  EXPECT_EQ(s.edges[1].topics.at(0), "/perception/obstacles");
  ASSERT_EQ(s.anomalies.size(), 2u);
  EXPECT_TRUE(s.anomalies[0].onset_time.has_value());
  EXPECT_DOUBLE_EQ(*s.anomalies[0].onset_time, 999.5);
  EXPECT_FALSE(s.anomalies[1].onset_time.has_value());
  EXPECT_TRUE(s.anomalies[1].informational);
  ASSERT_TRUE(s.latest_diagnosis.has_value());
  EXPECT_EQ(s.latest_diagnosis->root_cause, "sensor_node");
  EXPECT_DOUBLE_EQ(s.latest_diagnosis->confidence, 0.925);
  EXPECT_DOUBLE_EQ(s.latest_diagnosis->severity, 0.5);
  EXPECT_EQ(s.latest_diagnosis->weights.at("wS"), 0.25);
  EXPECT_EQ(s.latest_diagnosis->candidates.size(), 2u);
  EXPECT_EQ(s.latest_diagnosis->propagation_chain.size(), 2u);
  EXPECT_EQ(s.timeline.size(), 3u);
  EXPECT_EQ(s.timeline[2].event, "diagnosis");
}

TEST(Parse, MissingOptionalFieldsGetDefaults) {
  DashboardState s;
  const auto r = parse_dashboard_json(R"({"schema_version": 1})", s);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(s.system_status, "UNKNOWN");
  EXPECT_TRUE(s.nodes.empty());
  EXPECT_TRUE(s.edges.empty());
  EXPECT_FALSE(s.latest_diagnosis.has_value());
  EXPECT_TRUE(s.timeline.empty());
  EXPECT_EQ(s.counts.nodes, 0);

  // partially filled node / diagnosis without root cause / wrong types are tolerated
  const auto r2 = parse_dashboard_json(
      R"({"schema_version": 1, "system_status": "NORMAL", "nodes": [{"name": "a"}, {"status": "x"}, 5],
          "edges": [{"from": "a"}], "latest_diagnosis": {"confidence": "high"}, "counts": "nope"})", s);
  ASSERT_TRUE(r2.ok) << r2.error;
  ASSERT_EQ(s.nodes.size(), 1u);
  EXPECT_EQ(s.nodes[0].status, "NORMAL");
  EXPECT_TRUE(s.edges.empty());
  EXPECT_FALSE(s.latest_diagnosis.has_value());
  EXPECT_EQ(s.counts.nodes, 1);   // derived from the node list
}

TEST(Parse, InvalidJsonIsRejectedAndLeavesStateUntouched) {
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(kFullState, s).ok);
  const auto r = parse_dashboard_json("{not json", s);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("invalid JSON"), std::string::npos);
  EXPECT_EQ(s.system_status, "DEGRADED");     // untouched
  EXPECT_FALSE(parse_dashboard_json("[1,2,3]", s).ok);
  EXPECT_FALSE(parse_dashboard_json("", s).ok);
  EXPECT_FALSE(parse_dashboard_json("\"string\"", s).ok);
}

TEST(Parse, UnsupportedSchemaVersionIsRejected) {
  DashboardState s;
  const auto r = parse_dashboard_json(R"({"schema_version": 2, "system_status": "NORMAL"})", s);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("unsupported schema_version 2"), std::string::npos);
  EXPECT_FALSE(parse_dashboard_json(R"({"system_status": "NORMAL"})", s).ok);   // missing version
  EXPECT_FALSE(parse_dashboard_json(R"({"schema_version": "1"})", s).ok);       // wrong type
}

TEST(SharedState, ReplacementUpdateAndCounters) {
  SharedState shared(3.0);
  EXPECT_FALSE(shared.snapshot(0.0).has_state);
  EXPECT_EQ(shared.snapshot(0.0).connection, ConnectionStatus::kWaiting);

  ASSERT_TRUE(shared.update_from_json(kFullState, 100.0));
  auto snap = shared.snapshot(100.5);
  EXPECT_TRUE(snap.has_state);
  EXPECT_EQ(snap.state.system_status, "DEGRADED");
  EXPECT_EQ(snap.messages_ok, 1u);

  // a newer document fully replaces the old one (diagnosis cleared, nodes replaced)
  ASSERT_TRUE(shared.update_from_json(
      R"({"schema_version": 1, "system_status": "NORMAL", "nodes": [{"name": "only", "status": "NORMAL"}]})", 101.0));
  snap = shared.snapshot(101.1);
  EXPECT_EQ(snap.state.system_status, "NORMAL");
  ASSERT_EQ(snap.state.nodes.size(), 1u);
  EXPECT_FALSE(snap.state.latest_diagnosis.has_value());
  EXPECT_TRUE(snap.state.timeline.empty());

  // malformed input is counted, keeps the last good state, exposes the error
  EXPECT_FALSE(shared.update_from_json("garbage", 102.0));
  EXPECT_FALSE(shared.update_from_json(R"({"schema_version": 99})", 102.5));
  snap = shared.snapshot(102.6);
  EXPECT_TRUE(snap.has_state);
  EXPECT_EQ(snap.state.system_status, "NORMAL");
  EXPECT_EQ(snap.messages_rejected, 2u);
  EXPECT_FALSE(snap.last_error.empty());
  EXPECT_EQ(snap.connection, ConnectionStatus::kLive);   // last good message 1.6s ago
}

TEST(SharedState, StaleDetection) {
  SharedState shared(3.0);
  shared.update_from_json(kFullState, 100.0);
  EXPECT_EQ(shared.snapshot(102.9).connection, ConnectionStatus::kLive);
  auto snap = shared.snapshot(104.0);
  EXPECT_EQ(snap.connection, ConnectionStatus::kStale);
  EXPECT_NEAR(snap.age_sec, 4.0, 1e-9);
  // a fresh message recovers
  shared.update_from_json(kFullState, 104.5);
  EXPECT_EQ(shared.snapshot(104.6).connection, ConnectionStatus::kLive);
  // errors before any valid state show as error, not waiting
  SharedState bad(3.0);
  bad.update_from_json("{", 1.0);
  EXPECT_EQ(bad.snapshot(1.5).connection, ConnectionStatus::kError);
}

TEST(GraphView, LayeringFollowsReceivedEdgesOnly) {
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(kFullState, s).ok);
  const GraphView v = layout_graph(s);
  ASSERT_EQ(v.layers.size(), 4u);
  EXPECT_EQ(v.layers[0].nodes.at(0).name, "sensor_node");
  EXPECT_EQ(v.layers[1].nodes.at(0).name, "perception_node");
  EXPECT_EQ(v.layers[2].nodes.at(0).name, "localization_node");
  EXPECT_EQ(v.layers[3].nodes.at(0).name, "navigation_node");
  EXPECT_EQ(v.layers[0].nodes.at(0).status, "ROOT_CAUSE");
  EXPECT_TRUE(v.cycle_nodes.empty());

  // a different topology (fan-in) is laid out from the data, nothing assumed
  DashboardState t;
  ASSERT_TRUE(parse_dashboard_json(R"({"schema_version": 1,
      "nodes": [{"name": "lidar"}, {"name": "camera"}, {"name": "fusion"}, {"name": "planner"}, {"name": "lonely"}],
      "edges": [{"from": "lidar", "to": "fusion"}, {"from": "camera", "to": "fusion"},
                {"from": "fusion", "to": "planner"}, {"from": "lidar", "to": "planner"}]})", t).ok);
  const GraphView w = layout_graph(t);
  ASSERT_EQ(w.layers.size(), 3u);
  EXPECT_EQ(w.layers[0].nodes.size(), 3u);      // camera, lidar, lonely (sources / isolated)
  EXPECT_EQ(w.layers[1].nodes.at(0).name, "fusion");
  EXPECT_EQ(w.layers[2].nodes.at(0).name, "planner");   // longest path wins (depth 2, not 1)

  // a cycle does not crash and keeps every node visible
  DashboardState c;
  ASSERT_TRUE(parse_dashboard_json(R"({"schema_version": 1,
      "edges": [{"from": "a", "to": "b"}, {"from": "b", "to": "a"}, {"from": "src", "to": "a"}]})", c).ok);
  const GraphView cv = layout_graph(c);
  EXPECT_EQ(cv.cycle_nodes.size(), 2u);
  std::size_t total = 0;
  for (const auto& l : cv.layers) total += l.nodes.size();
  EXPECT_EQ(total, 3u);
  EXPECT_EQ(cv.layers[0].nodes.at(0).name, "src");
  EXPECT_EQ(cv.layers[0].nodes.at(0).status, "UNKNOWN");   // referenced only by edges

  EXPECT_TRUE(layout_graph(DashboardState{}).layers.empty());
}

TEST(DiagnosisView, TransformsScoresChainAndEvidence) {
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(kFullState, s).ok);
  const DiagnosisView v = diagnosis_view(s);
  EXPECT_TRUE(v.present);
  EXPECT_EQ(v.root_cause, "sensor_node");
  EXPECT_EQ(v.confidence, "0.925");
  ASSERT_EQ(v.scores.size(), 4u);
  EXPECT_EQ(v.scores[0].first, "Dependency");
  EXPECT_DOUBLE_EQ(v.scores[3].second, 0.5);
  EXPECT_EQ(v.chain, "sensor_node -> perception_node");
  ASSERT_EQ(v.evidence_lines.size(), 3u);   // separator-only lines dropped, text kept verbatim
  EXPECT_EQ(v.evidence_lines[0], "ROOT CAUSE DIAGNOSIS: sensor_node (Confidence: 0.925)");
  EXPECT_EQ(v.evidence_lines[2], "  - Dependency D: 1.00");
  EXPECT_EQ(v.candidates.size(), 2u);

  const DiagnosisView none = diagnosis_view(DashboardState{});
  EXPECT_FALSE(none.present);
  EXPECT_EQ(none.root_cause, "None");
}

TEST(Helpers, MarkersAndClock) {
  EXPECT_EQ(node_marker("ROOT_CAUSE"), "[*]");
  EXPECT_EQ(node_marker("ANOMALOUS"), "[!]");
  EXPECT_EQ(node_marker("MISSING"), "[X]");
  EXPECT_EQ(node_marker("NORMAL"), "[ ]");
  EXPECT_EQ(node_marker("whatever"), "[?]");
  EXPECT_EQ(format_clock(0.0), "--:--:--.--");
  const std::string c = format_clock(1700000000.25);
  EXPECT_EQ(c.size(), 11u);
  EXPECT_EQ(c.substr(8), ".25");
}

TEST(Render, AllStatesRenderWithoutThrowing) {
  UiOptions opt;
  // no data yet
  SharedState empty(3.0);
  std::string txt = render_to_text(render_dashboard(empty.snapshot(0.0), opt, 0.0));
  EXPECT_NE(txt.find("WAITING FOR DIAGNOSTIC MONITOR"), std::string::npos);
  EXPECT_NE(txt.find("No dashboard state"), std::string::npos);

  // normal, no anomalies, no diagnosis
  SharedState normal(3.0);
  normal.update_from_json(R"({"schema_version": 1, "system_status": "NORMAL",
      "nodes": [{"name": "a", "status": "NORMAL"}, {"name": "b", "status": "NORMAL"}],
      "edges": [{"from": "a", "to": "b", "topics": ["/t"]}], "rca_window_sec": 6})", 10.0);
  txt = render_to_text(render_dashboard(normal.snapshot(10.5), opt, 10.5));
  EXPECT_NE(txt.find("SYSTEM NORMAL"), std::string::npos);
  EXPECT_NE(txt.find("None"), std::string::npos);
  EXPECT_NE(txt.find("a -> b"), std::string::npos);
  EXPECT_NE(txt.find("ROS LIVE"), std::string::npos);

  // degraded with diagnosis
  SharedState degraded(3.0);
  degraded.update_from_json(kFullState, 20.0);
  for (Focus f : {Focus::kGraph, Focus::kDiagnosis, Focus::kTimeline}) {
    opt.focus = f;
    txt = render_to_text(render_dashboard(degraded.snapshot(20.5), opt, 20.5));
    EXPECT_NE(txt.find("SYSTEM DEGRADED"), std::string::npos);
    EXPECT_NE(txt.find("sensor_node"), std::string::npos);
    EXPECT_NE(txt.find("Confidence: 0.925"), std::string::npos);
    EXPECT_NE(txt.find("Dependency"), std::string::npos);
    EXPECT_NE(txt.find("sensor_node -> perception_node"), std::string::npos);
    EXPECT_NE(txt.find("diagnosis"), std::string::npos);   // timeline row
  }
  // stale
  txt = render_to_text(render_dashboard(degraded.snapshot(30.0), opt, 30.0));
  EXPECT_NE(txt.find("ROS STATE STALE"), std::string::npos);

  // malformed input after good state keeps rendering the good state and reports the error count
  degraded.update_from_json("{{{", 31.0);
  txt = render_to_text(render_dashboard(degraded.snapshot(31.0), opt, 31.0));
  EXPECT_NE(txt.find("rejected 1"), std::string::npos);
  EXPECT_NE(txt.find("sensor_node"), std::string::npos);

  // tiny terminal must not throw either
  EXPECT_NO_THROW(render_to_text(render_dashboard(degraded.snapshot(31.0), opt, 31.0), 40, 12));
}

// ---------------------------------------------------------------------------
// Backend label and the ROS-graph-membership vs runtime-liveness distinction.
// ---------------------------------------------------------------------------

TEST(Parse, SimulatorLabelIsReadAndDefaultsToUnknown) {
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(kFullState, s).ok);
  EXPECT_EQ(s.simulator, "unknown");  // kFullState carries no backend label

  const char* gz = R"JSON({"schema_version": 1, "timestamp": 1.0, "simulator": "gazebo",
    "system_status": "NORMAL", "nodes": [], "edges": [], "anomalies": [], "timeline": []})JSON";
  DashboardState g;
  ASSERT_TRUE(parse_dashboard_json(gz, g).ok);
  EXPECT_EQ(g.simulator, "gazebo");

  const char* sim = R"JSON({"schema_version": 1, "timestamp": 1.0, "simulator": "rca_sim",
    "system_status": "NORMAL", "nodes": [], "edges": [], "anomalies": [], "timeline": []})JSON";
  DashboardState r;
  ASSERT_TRUE(parse_dashboard_json(sim, r).ok);
  EXPECT_EQ(r.simulator, "rca_sim");
}

TEST(Parse, GraphMembershipAndLivenessAreSeparateObservations) {
  const char* crashed = R"JSON({
    "schema_version": 1, "timestamp": 10.0, "simulator": "gazebo", "system_status": "DEGRADED",
    "nodes": [
      {"name": "perception_node", "status": "MISSING", "in_ros_graph": true, "liveness": "STARVED"},
      {"name": "lidar_node", "status": "NORMAL", "in_ros_graph": true, "liveness": "ALIVE"},
      {"name": "ghost_node", "status": "ANOMALOUS", "in_ros_graph": false, "liveness": "UNKNOWN"}
    ],
    "edges": [], "anomalies": [], "timeline": []})JSON";
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(crashed, s).ok);
  ASSERT_EQ(s.nodes.size(), 3u);
  // A crashed process is starved long before the DDS lease removes it from the graph.
  EXPECT_TRUE(s.nodes[0].in_ros_graph);
  EXPECT_EQ(s.nodes[0].liveness, "STARVED");
  EXPECT_EQ(s.nodes[1].liveness, "ALIVE");
  EXPECT_FALSE(s.nodes[2].in_ros_graph);
  EXPECT_EQ(s.nodes[2].liveness, "UNKNOWN");
}

TEST(Parse, LivenessFieldsDefaultSafelyWhenAbsent) {
  DashboardState s;
  ASSERT_TRUE(parse_dashboard_json(kFullState, s).ok);
  for (const auto& n : s.nodes) {
    EXPECT_TRUE(n.in_ros_graph);
    EXPECT_EQ(n.liveness, "UNKNOWN");
  }
}

TEST(Render, BackendAndLivenessAreVisibleAndNoGroundTruthIsShown) {
  const char* gz = R"JSON({
    "schema_version": 1, "timestamp": 10.0, "simulator": "gazebo", "monitor_uptime_sec": 30.0,
    "system_status": "DEGRADED",
    "counts": {"nodes": 2, "edges": 1, "topics": 2, "anomalies_in_window": 1, "anomalous_nodes": 1},
    "nodes": [
      {"name": "lidar_node", "status": "ROOT_CAUSE", "anomaly_count": 2, "max_severity": 1.0,
       "last_metric": "latency", "in_ros_graph": true, "liveness": "ALIVE"},
      {"name": "perception_node", "status": "MISSING", "in_ros_graph": true, "liveness": "STARVED"}
    ],
    "edges": [{"from": "lidar_node", "to": "perception_node", "topics": ["/sim/lidar/scan"]}],
    "anomalies": [], "timeline": []})JSON";
  SharedState shared(3.0);
  ASSERT_TRUE(shared.update_from_json(gz, 100.0));
  const Snapshot snap = shared.snapshot(100.2);

  UiOptions opt;
  const std::string out = render_to_text(render_dashboard(snap, opt, 100.2), 170, 46);

  EXPECT_NE(out.find("gazebo"), std::string::npos);
  EXPECT_NE(out.find("STARVED"), std::string::npos);
  EXPECT_NE(out.find("in graph"), std::string::npos);
  // The TUI must never display evaluation ground truth in any form.
  for (const char* forbidden : {"ground_truth", "true_root", "injected", "accepted_roots",
                                "observable_chain", "physical_chain", "scenario"}) {
    EXPECT_EQ(out.find(forbidden), std::string::npos) << forbidden;
  }
}

// main() is provided by gtest_main via ament_add_gtest.
