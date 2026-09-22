// FTXUI rendering of the dashboard snapshot (one screen, three panels).
#pragma once

#include <ftxui/dom/elements.hpp>

#include "diagnostic_tui/dashboard_state.hpp"

namespace diagnostic_tui {

enum class Focus { kGraph, kDiagnosis, kTimeline };

struct UiOptions {
  Focus focus = Focus::kGraph;
  int evidence_scroll = 0;     // first evidence line shown in the diagnosis panel
  int timeline_scroll = 0;     // lines scrolled back from the newest timeline event
  int timeline_rows = 12;      // visible timeline rows
  int evidence_rows = 12;      // visible evidence rows
};

ftxui::Element render_header(const Snapshot& snap, double now_sec);
ftxui::Element render_graph(const Snapshot& snap, bool focused);
ftxui::Element render_diagnosis(const Snapshot& snap, const UiOptions& opt, bool focused);
ftxui::Element render_timeline(const Snapshot& snap, const UiOptions& opt, bool focused);
ftxui::Element render_footer(const Snapshot& snap, const UiOptions& opt);
ftxui::Element render_dashboard(const Snapshot& snap, const UiOptions& opt, double now_sec);

}  // namespace diagnostic_tui
