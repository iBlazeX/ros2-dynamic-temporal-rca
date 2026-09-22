#include "diagnostic_tui/render.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <ftxui/screen/color.hpp>

namespace diagnostic_tui {

using namespace ftxui;

namespace {

constexpr int kRightPanelWidth = 66;     // diagnosis panel: fixed so the graph keeps its space
constexpr std::size_t kEvidenceWidth = 60;

std::string fmt(double v, int prec = 2) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(prec) << v;
  return o.str();
}

std::string pad(std::string s, std::size_t w) {
  if (s.size() > w) return s.substr(0, w);
  s.append(w - s.size(), ' ');
  return s;
}

Color status_color(const std::string& status) {
  if (status == "ROOT_CAUSE") return Color::Red;
  if (status == "ANOMALOUS") return Color::Yellow;
  if (status == "MISSING") return Color::Magenta;
  if (status == "NORMAL") return Color::Green;
  return Color::GrayDark;
}

Color system_color(const std::string& s) {
  if (s == "NORMAL") return Color::Green;
  if (s == "DEGRADED") return Color::Red;
  if (s == "RECOVERING") return Color::Yellow;
  return Color::GrayLight;
}

Element panel(const std::string& title, Element body, bool focused) {
  Element t = text(" " + title + " ") | bold;
  if (focused) t = t | inverted;
  return window(t, body) | (focused ? borderStyled(HEAVY) : borderStyled(LIGHT));
}

Element node_box(const NodeInfo& n) {
  const Color c = status_color(n.status);
  Elements lines;
  Element title = text(node_marker(n.status) + " " + n.name) | bold | color(c);
  if (n.status == "ROOT_CAUSE") title = title | inverted;
  lines.push_back(title);
  if (n.status == "ANOMALOUS" || n.status == "ROOT_CAUSE") {
    lines.push_back(text(" " + n.last_metric + "  sev " + fmt(n.max_severity) + "  x" +
                         std::to_string(n.anomaly_count)) | color(c));
  } else if (n.status == "MISSING") {
    lines.push_back(text(" no data (runtime starvation)") | color(c));
  } else if (n.status == "NORMAL") {
    lines.push_back(text(" nominal") | dim);
  } else {
    lines.push_back(text(" no telemetry") | dim);
  }
  // ROS graph membership and runtime liveness are different observations and
  // are shown as such: a crashed node is STARVED long before it leaves the graph.
  const std::string graph_txt = n.in_ros_graph ? "in graph" : "not in graph";
  const Color live_c = (n.liveness == "STARVED") ? Color::Magenta
                     : (n.liveness == "ALIVE") ? Color::GrayLight : Color::GrayDark;
  lines.push_back(text(" " + graph_txt + " / " + n.liveness) | color(live_c) | dim);
  return vbox(lines) | borderStyled(c);
}

}  // namespace

Element render_header(const Snapshot& snap, double /*now_sec*/) {
  std::string conn;
  Color conn_color = Color::GrayLight;
  switch (snap.connection) {
    case ConnectionStatus::kLive:    conn = "ROS LIVE";  conn_color = Color::Green; break;
    case ConnectionStatus::kStale:   conn = "ROS STATE STALE"; conn_color = Color::Yellow; break;
    case ConnectionStatus::kError:   conn = "BAD INPUT"; conn_color = Color::Red; break;
    case ConnectionStatus::kWaiting: conn = "WAITING FOR DIAGNOSTIC MONITOR"; break;
  }
  Elements row;
  row.push_back(text(" Dynamic Dependency-Aware Temporal RCA  |  ROS 2 diagnostic console ") | bold);
  row.push_back(filler());
  row.push_back(text(" " + conn + " ") | bold | color(conn_color));
  Elements info;
  if (snap.has_state) {
    const auto& s = snap.state;
    info.push_back(text(" SYSTEM " + s.system_status + " ") | bold | color(system_color(s.system_status)) | inverted);
    info.push_back(text("  backend " + s.simulator + " ") | dim);
    info.push_back(text("  nodes " + std::to_string(s.counts.nodes) +
                        "  edges " + std::to_string(s.counts.edges) +
                        "  topics " + std::to_string(s.counts.topics)));
    info.push_back(text("  anomalies(window) " + std::to_string(s.counts.anomalies_in_window) +
                        "  anomalous nodes " + std::to_string(s.counts.anomalous_nodes)) |
                   color(s.counts.anomalies_in_window ? Color::Yellow : Color::GrayLight));
    info.push_back(filler());
    info.push_back(text("state age " + fmt(snap.age_sec, 1) + "s  monitor up " +
                        fmt(s.monitor_uptime_sec, 0) + "s  msgs " + std::to_string(snap.messages_ok) +
                        (snap.messages_rejected ? " (rejected " + std::to_string(snap.messages_rejected) + ")" : "")) | dim);
  } else {
    info.push_back(text(" No dashboard state received yet. Start: ros2 launch rca_test_system system.launch.py ") | dim);
    if (!snap.last_error.empty()) {
      info.push_back(filler());
      info.push_back(text(" last error: " + snap.last_error + " ") | color(Color::Red));
    }
  }
  return vbox({hbox(row), hbox(info)}) | border;
}

Element render_graph(const Snapshot& snap, bool focused) {
  if (!snap.has_state || snap.state.nodes.empty()) {
    return panel("DEPENDENCY GRAPH (discovered at runtime)",
                 vbox({text(""), text("  waiting for graph discovery ...") | dim, filler()}), focused);
  }
  const GraphView view = layout_graph(snap.state);
  Elements columns;
  for (std::size_t i = 0; i < view.layers.size(); ++i) {
    Elements boxes;
    for (const auto& n : view.layers[i].nodes) boxes.push_back(node_box(n));
    columns.push_back(vbox(boxes) | vcenter);
    if (i + 1 < view.layers.size()) {
      columns.push_back(vbox({filler(), text(" ──▶ ") | bold, filler()}));
    }
  }
  Elements edge_lines;
  for (const auto& e : view.edges) {
    std::string topics;
    for (std::size_t k = 0; k < e.topics.size(); ++k) topics += (k ? ", " : "") + e.topics[k];
    edge_lines.push_back(text("  " + e.from + " -> " + e.to + "   via " + topics) | dim);
  }
  if (!view.cycle_nodes.empty()) {
    edge_lines.push_back(text("  (cycle detected; nodes shown in trailing column)") | color(Color::Yellow));
  }
  Element legend = hbox({
      text("  [ ] normal ") | color(Color::Green), text(" [!] anomalous ") | color(Color::Yellow),
      text(" [*] probable root cause ") | color(Color::Red), text(" [X] starved ") | color(Color::Magenta),
      text(" [?] unknown") | color(Color::GrayDark),
      text("   | each node shows ROS graph membership / runtime liveness") | dim});
  Element body = vbox({
      text(""),
      hbox(columns) | hcenter,
      text(""),
      separator(),
      text("  edges (dynamic ROS 2 graph discovery):") | bold,
      vbox(edge_lines),
      filler(),
      legend,
  });
  return panel("DEPENDENCY GRAPH (discovered at runtime)", body, focused);
}

Element render_diagnosis(const Snapshot& snap, const UiOptions& opt, bool focused) {
  Elements lines;
  if (!snap.has_state) {
    lines.push_back(text(""));
    lines.push_back(text("  no data") | dim);
    return panel("ROOT CAUSE DIAGNOSIS", vbox(lines), focused);
  }
  const DiagnosisView v = diagnosis_view(snap.state);
  lines.push_back(text("  ROOT CAUSE") | bold);
  if (!v.present) {
    lines.push_back(text("  None") | bold | color(Color::Green));
    lines.push_back(text(""));
    lines.push_back(text("  No anomalies in the RCA window (" + fmt(snap.state.rca_window_sec, 0) + "s).") | dim);
    lines.push_back(text("  System operating nominally.") | dim);
    return panel("ROOT CAUSE DIAGNOSIS", vbox(lines), focused);
  }
  lines.push_back(text("  " + v.root_cause) | bold | color(Color::Red));
  lines.push_back(text(""));
  lines.push_back(text("  Confidence: " + v.confidence) | bold);
  lines.push_back(text(""));
  for (const auto& sc : v.scores) {
    lines.push_back(hbox({text("  " + pad(sc.first, 12)),
                          gauge(static_cast<float>(sc.second)) | size(WIDTH, EQUAL, 14) | color(Color::Cyan),
                          text(" " + fmt(sc.second))}));
  }
  lines.push_back(text(""));
  lines.push_back(text("  Propagation: " + v.chain.substr(0, 60)) | color(Color::Yellow));
  if (!v.candidates.empty()) {
    lines.push_back(text("  Ranking:") | bold);
    int shown = 0;
    for (const auto& c : v.candidates) {
      if (shown++ >= 4) break;
      lines.push_back(text("   " + std::to_string(shown) + ". " + pad(c.node, 18) + " R=" + fmt(c.total, 3)) |
                      (shown == 1 ? color(Color::Red) : dim));
    }
  }
  lines.push_back(separator());
  lines.push_back(text("  Evidence (from diagnostic monitor):") | bold);
  const int total = static_cast<int>(v.evidence_lines.size());
  const int start = std::clamp(opt.evidence_scroll, 0, std::max(0, total - 1));
  const int end = std::min(total, start + opt.evidence_rows);
  for (int i = start; i < end; ++i) {
    lines.push_back(text("  " + v.evidence_lines[static_cast<std::size_t>(i)].substr(0, kEvidenceWidth)) | dim);
  }
  if (total > opt.evidence_rows) {
    lines.push_back(text("  [" + std::to_string(start + 1) + "-" + std::to_string(end) + "/" +
                         std::to_string(total) + "]  press D then ↑/↓") | color(Color::GrayDark));
  }
  return panel("ROOT CAUSE DIAGNOSIS", vbox(lines), focused);
}

Element render_timeline(const Snapshot& snap, const UiOptions& opt, bool focused) {
  Elements rows;
  if (!snap.has_state || snap.state.timeline.empty()) {
    rows.push_back(text("  no events yet") | dim);
    return panel("TEMPORAL TIMELINE (onset-ordered evidence; ordering supports, does not prove, causality)",
                 vbox(rows), focused);
  }
  const auto& tl = snap.state.timeline;
  const int total = static_cast<int>(tl.size());
  const int back = std::clamp(opt.timeline_scroll, 0, std::max(0, total - opt.timeline_rows));
  const int end = total - back;
  const int start = std::max(0, end - opt.timeline_rows);
  for (int i = start; i < end; ++i) {
    const auto& e = tl[static_cast<std::size_t>(i)];
    Color c = Color::GrayLight;
    if (e.event == "diagnosis") c = Color::Red;
    else if (e.event == "window_cleared" || e.event == "graph_discovered") c = Color::Green;
    else if (e.event == "out_of_order") c = Color::GrayDark;
    else if (e.severity >= 0.7) c = Color::Yellow;
    rows.push_back(hbox({text(format_clock(e.t) + "  ") | dim,
                         text(pad(e.node, 20) + " ") | bold | color(c),
                         text(pad(e.event, 17) + " ") | color(c),
                         text(e.severity > 0.0 ? "sev " + fmt(e.severity) + "  " : "          ") | dim,
                         text(e.message) | dim}));
  }
  if (total > opt.timeline_rows) {
    rows.push_back(text("  [" + std::to_string(start + 1) + "-" + std::to_string(end) + "/" +
                        std::to_string(total) + "]  press T then ↑/↓") | color(Color::GrayDark));
  }
  return panel("TEMPORAL TIMELINE (onset-ordered evidence; ordering supports, does not prove, causality)",
               vbox(rows), focused);
}

Element render_footer(const Snapshot& /*snap*/, const UiOptions& opt) {
  auto key = [&](const std::string& k, const std::string& label, bool active) {
    Element e = text(" [" + k + "] " + label + " ");
    return active ? (e | inverted) : e;
  };
  return hbox({key("R", "Refresh", false),
               key("G", "Graph", opt.focus == Focus::kGraph),
               key("D", "Diagnosis", opt.focus == Focus::kDiagnosis),
               key("T", "Timeline", opt.focus == Focus::kTimeline),
               text(" [↑/↓] Scroll focused panel "), key("Q", "Quit", false), filler(),
               text(" visualisation layer only: all RCA logic runs in diagnostic_monitor ") | dim});
}

Element render_dashboard(const Snapshot& snap, const UiOptions& opt, double now_sec) {
  Element left = render_graph(snap, opt.focus == Focus::kGraph) | flex;
  Element right = render_diagnosis(snap, opt, opt.focus == Focus::kDiagnosis) | size(WIDTH, EQUAL, kRightPanelWidth);
  Element body = hbox({left | xflex, right}) | yflex;
  Element bottom = render_timeline(snap, opt, opt.focus == Focus::kTimeline);
  return vbox({render_header(snap, now_sec), body, bottom, render_footer(snap, opt)});
}

}  // namespace diagnostic_tui
