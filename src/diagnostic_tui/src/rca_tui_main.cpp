// rca_tui: live terminal dashboard for the diagnostic monitor.
//
//   ROS 2 thread  : rclcpp executor -> subscription callback -> SharedState.update()
//   UI thread     : FTXUI ScreenInteractive::Loop -> render(SharedState.snapshot())
//
// The ROS callback never renders; it parses, stores and posts a redraw event.
// The UI thread never blocks on ROS; it only reads a copy of the shared state.
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "diagnostic_tui/dashboard_state.hpp"
#include "diagnostic_tui/render.hpp"

namespace {

double wall_now() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

class DashboardSubscriber : public rclcpp::Node {
 public:
  DashboardSubscriber(std::shared_ptr<diagnostic_tui::SharedState> state,
                      ftxui::ScreenInteractive* screen)
      : rclcpp::Node("rca_tui"), state_(std::move(state)), screen_(screen) {
    // Match the monitor's latched publisher so a late-starting TUI gets the last state.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable().transient_local();
    sub_ = create_subscription<std_msgs::msg::String>(
        "/rca/dashboard_state", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          state_->update_from_json(msg->data, wall_now());
          if (screen_) screen_->PostEvent(ftxui::Event::Custom);   // thread-safe redraw request
        });
  }

 private:
  std::shared_ptr<diagnostic_tui::SharedState> state_;
  ftxui::ScreenInteractive* screen_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto state = std::make_shared<diagnostic_tui::SharedState>(/*stale_after_sec=*/3.0);
  auto screen = ftxui::ScreenInteractive::Fullscreen();
  auto node = std::make_shared<DashboardSubscriber>(state, &screen);

  // ROS 2 thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() { executor.spin(); });

  // 1 Hz ticker so "state age" / staleness update even when nothing is received
  std::atomic<bool> running{true};
  std::thread ticker([&]() {
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (running.load()) screen.PostEvent(ftxui::Event::Custom);
    }
  });

  // UI thread (this thread)
  diagnostic_tui::UiOptions opt;
  auto renderer = ftxui::Renderer([&]() {
    return diagnostic_tui::render_dashboard(state->snapshot(wall_now()), opt, wall_now());
  });
  auto component = ftxui::CatchEvent(renderer, [&](ftxui::Event e) {
    using diagnostic_tui::Focus;
    if (e == ftxui::Event::Character('q') || e == ftxui::Event::Character('Q') || e == ftxui::Event::Escape) {
      screen.Exit();
      return true;
    }
    if (e == ftxui::Event::Character('g') || e == ftxui::Event::Character('G')) { opt.focus = Focus::kGraph; return true; }
    if (e == ftxui::Event::Character('d') || e == ftxui::Event::Character('D')) { opt.focus = Focus::kDiagnosis; return true; }
    if (e == ftxui::Event::Character('t') || e == ftxui::Event::Character('T')) { opt.focus = Focus::kTimeline; return true; }
    if (e == ftxui::Event::Character('r') || e == ftxui::Event::Character('R')) {
      opt.evidence_scroll = 0;
      opt.timeline_scroll = 0;
      return true;   // returning true triggers a redraw with the latest snapshot
    }
    if (e == ftxui::Event::ArrowUp || e == ftxui::Event::ArrowDown) {
      const int delta = (e == ftxui::Event::ArrowUp) ? -1 : 1;
      if (opt.focus == Focus::kDiagnosis) opt.evidence_scroll = std::max(0, opt.evidence_scroll + delta);
      if (opt.focus == Focus::kTimeline) opt.timeline_scroll = std::max(0, opt.timeline_scroll - delta);
      return true;
    }
    return false;  // Event::Custom and everything else: just redraw
  });

  screen.Loop(component);

  running.store(false);
  rclcpp::shutdown();          // stops executor.spin()
  ros_thread.join();
  ticker.join();
  return 0;
}
