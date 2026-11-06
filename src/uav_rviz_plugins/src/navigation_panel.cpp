#include "uav_rviz_plugins/navigation_panel.hpp"

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QJsonDocument>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
namespace uav_rviz_plugins {
NavigationPanel::NavigationPanel(QWidget* parent) : Panel(parent) {
  setMinimumWidth(325);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto* content = new QWidget;
  content->setStyleSheet(
      "QLabel{font-size:12px;} QGroupBox{font-size:12px;} QPushButton{padding:3px;}");
  auto* layout = new QVBoxLayout(content);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  layout->setContentsMargins(8, 6, 8, 6);
  layout->setSpacing(5);
  auto label = [&](const QString& text, const char* name) {
    auto* item = new QLabel(text);
    item->setObjectName(name);
    item->setWordWrap(true);
    item->setTextFormat(Qt::PlainText);
    layout->addWidget(item);
    return item;
  };
  label("SCUT UAV · 导航工作台", "title")
      ->setStyleSheet("font-size:17px;font-weight:bold;color:#167b91");
  mission_ = label("导航：等待连接", "mission_status");
  px4_ = label("PX4：等待连接", "px4_status");
  position_ = label("位置 / 速度：—", "position_status");
  health_ = label("定位 / 点云：—", "health_status");
  planner_ = label("规划耗时：—", "planning_status");
  auto* goal_box = new QGroupBox("导航目标 · odom / 米");
  auto* form = new QFormLayout(goal_box);
  auto spin = [](const char* name, double low, double high, double value) {
    auto* widget = new QDoubleSpinBox;
    widget->setObjectName(name);
    widget->setRange(low, high);
    widget->setDecimals(2);
    widget->setSingleStep(.1);
    widget->setValue(value);
    return widget;
  };
  x_ = spin("goal_x", -10000, 10000, 0);
  y_ = spin("goal_y", -10000, 10000, 2);
  z_ = spin("goal_z", -1000, 1000, 2.5);
  form->addRow("X", x_);
  form->addRow("Y", y_);
  form->addRow("Z 高度", z_);
  layout->addWidget(goal_box);
  auto* goal_grid = new QGridLayout;
  auto button = [&](QGridLayout* grid, const std::string& key, const QString& text, int row,
                    int col) {
    auto* b = new QPushButton(text);
    b->setObjectName(key.c_str());
    buttons_[key] = b;
    grid->addWidget(b, row, col);
    return b;
  };
  connect(button(goal_grid, "select_point", "地图选点", 0, 0), &QPushButton::clicked, this, [this] {
    if (!getDisplayContext()) return;
    auto* tools = getDisplayContext()->getToolManager();
    for (int i = 0; i < tools->numTools(); ++i)
      if (tools->getTool(i)->getClassId() == "uav_rviz_plugins/GoalPlaneTool") {
        tools->getTool(i)->getPropertyContainer()->subProp("Height")->setValue(z_->value());
        tools->setCurrentTool(tools->getTool(i));
        finish("点击高度平面选择目标，蓝框外和空白区域也可选点", true);
        return;
      }
  });
  connect(z_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          [this](double) { syncGoalPlaneHeight(); });
  connect(button(goal_grid, "stage", "设置目标", 0, 1), &QPushButton::clicked, this,
          [this] { setGoal(false); });
  connect(button(goal_grid, "preview", "预览局部规划", 1, 0), &QPushButton::clicked, this,
          [this] { setGoal(true); });
  auto* start = button(goal_grid, "start", "开始导航", 1, 1);
  start->setStyleSheet(
      "QPushButton:enabled{background:#197f91;color:white;font-weight:bold;padding:6px}");
  connect(start, &QPushButton::clicked, this, [this] { setGoal(false, true); });
  connect(button(goal_grid, "pause", "暂停 / 制动", 2, 0), &QPushButton::clicked, this,
          [this] { trigger("pause"); });
  connect(button(goal_grid, "resume", "恢复导航", 2, 1), &QPushButton::clicked, this,
          [this] { trigger("resume"); });
  connect(button(goal_grid, "cancel", "取消导航", 3, 0), &QPushButton::clicked, this,
          [this] { trigger("cancel"); });
  connect(button(goal_grid, "current_height", "采用当前高度", 3, 1), &QPushButton::clicked, this,
          [this] { z_->setValue(pz_); });
  layout->addLayout(goal_grid);
  auto* pilot_box = new QGroupBox("PX4 仿真飞手 · 显式操作");
  auto* pg = new QGridLayout(pilot_box);
  height_ = spin("takeoff_height", .5, 4, 2.5);
  pg->addWidget(new QLabel("上升高度 / m"), 0, 0);
  pg->addWidget(height_, 0, 1);
  for (const auto& item :
       std::vector<std::tuple<std::string, QString, int, int>>{{"arm", "解锁", 1, 0},
                                                               {"disarm", "地面上锁", 1, 1},
                                                               {"takeoff", "解锁并起飞", 2, 0},
                                                               {"hold", "PX4 悬停", 2, 1},
                                                               {"land", "降落", 3, 0}}) {
    auto key = std::get<0>(item);
    auto* b = button(pg, key, std::get<1>(item), std::get<2>(item), std::get<3>(item));
    connect(b, &QPushButton::clicked, this, [this, key] { pilotCommand(key); });
  }
  buttons_["land"]->setStyleSheet("QPushButton:enabled{color:#b64b26;font-weight:bold}");
  connect(button(pg, "reset_control_fault", "地面故障复位", 3, 1), &QPushButton::clicked, this,
          [this] { trigger("reset_control_fault"); });
  layout->addWidget(pilot_box);
  pilot_ = label("飞手操作：等待状态", "pilot_status");
  result_ = label("地图选点或输入 XYZ → 预览 / 开始。预览不会执行飞行。", "result");
  auto* views = new QGridLayout;
  for (const auto& item : std::vector<std::pair<std::string, QString>>{
           {"overview", "全场"}, {"follow", "跟随机体"}, {"top", "俯视"}}) {
    auto* b = new QPushButton(item.second);
    views->addWidget(b, 0, views->count());
    connect(b, &QPushButton::clicked, this, [this, key = item.first] { view(key); });
  }
  layout->addLayout(views);
  auto* layers = new QGridLayout;
  for (const auto& item : std::vector<std::pair<QString, QString>>{
           {"地图显隐", "Inflated obstacles"}, {"点云显隐", "Odin scan (odom)"}}) {
    auto* b = new QPushButton(item.first);
    layers->addWidget(b, 0, layers->count());
    connect(b, &QPushButton::clicked, this, [this, name = item.second] {
      auto* manager = dynamic_cast<rviz_common::VisualizationManager*>(getDisplayContext());
      if (!manager) return;
      auto* group = manager->getRootDisplayGroup();
      for (int i = 0; i < group->numDisplays(); ++i) {
        auto* display = group->getDisplayAt(i);
        if (display->getName() == name) display->setEnabled(!display->isEnabled());
      }
    });
  }
  layout->addLayout(layers);
  label(
      "黄：A* "
      "搜索（碰撞时触发）　蓝：初始轨迹\n粉：优化候选　青：已接受轨迹　绿：实际航迹\n紫：暂存目标；"
      "所有轨迹位于 odom。",
      "legend")
      ->setStyleSheet("font-size:11px;color:#65727c");
  layout->addStretch();
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this] {
    if (node_ && rclcpp::ok()) executor_.spin_some(std::chrono::milliseconds(4));
    if (busy_ && operation_age_.elapsed() > 6000) {
      ++serial_;
      if (cancel_) cancel_();
      cancel_ = {};
      finish("请求超时；以当前任务/飞控状态为准，不自动重试", false);
    }
    updateButtons();
  });
  updateButtons();
}
NavigationPanel::~NavigationPanel() {
  timer_->stop();
  if (cancel_) cancel_();
  if (node_) executor_.remove_node(node_);
}
void NavigationPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("scut_navigation_panel");
  executor_.add_node(node_);
  goal_client_ = node_->create_client<Goal>("/navigation/set_goal");
  pilot_client_ = node_->create_client<Pilot>("/navigation/pilot_command");
  for (const auto* action : {"start", "pause", "resume", "cancel", "reset_control_fault"})
    clients_[action] = node_->create_client<Trigger>(std::string("/navigation/") + action);
  diagnostics_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/control_diagnostics", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(m->data));
        if (!doc.isObject()) return;
        diagnostics_ = doc.object();
        connected_ = true;
        diag_age_.restart();
        const std::map<int, QString> modes{{0, "MANUAL"}, {2, "POSCTL"},    {4, "HOLD"},
                                           {5, "RTL"},    {14, "OFFBOARD"}, {17, "TAKEOFF"},
                                           {18, "LAND"}};
        int mode = diagnostics_["px4_mode"].toInt(-1);
        px4_->setText(QString("PX4：%1 · %2 · %3\n控制桥：%4 · 对齐 %5 · EV %6")
                          .arg(modes.count(mode) ? modes.at(mode) : QString::number(mode))
                          .arg(diagnostics_["armed"].toBool() ? "已解锁" : "已上锁")
                          .arg(diagnostics_["landed"].toBool() ? "地面" : "空中")
                          .arg(diagnostics_["phase"].toString())
                          .arg(diagnostics_["aligned"].toBool() ? "✓" : "—")
                          .arg(diagnostics_["ev_pos"].toBool() ? "✓" : "—"));
        if (!diagnostics_["control_fault"].toString().isEmpty())
          px4_->setText(px4_->text() + "\n故障：" + diagnostics_["control_fault"].toString());
        pilot_->setText(diagnostics_["pilot_message"].toString() + "\n" +
                        diagnostics_["command_ack"].toString());
      });
  status_sub_ = node_->create_subscription<uav_interfaces::msg::NavigationStatus>(
      "/navigation/status", rclcpp::QoS(1).transient_local(),
      [this](uav_interfaces::msg::NavigationStatus::ConstSharedPtr m) {
        phase_ = QString::fromStdString(m->phase);
        have_status_ = true;
        status_age_.restart();
        mission_->setText(QString("导航：%1 · 航点 %2/%3\n%4")
                              .arg(phase_)
                              .arg(std::min(m->waypoint_index + 1, m->waypoint_count))
                              .arg(m->waypoint_count)
                              .arg(QString::fromStdString(m->reason)));
      });
  health_sub_ = node_->create_subscription<uav_interfaces::msg::NavigationHealth>(
      "/navigation/health", 10, [this](uav_interfaces::msg::NavigationHealth::ConstSharedPtr m) {
        health_age_.restart();
        source_healthy_ = m->odometry_valid && m->cloud_valid;
        health_->setText(QString("来源：%1\n定位 %2 (%3 s) · 点云 %4 (%5 s)")
                             .arg(QString::fromStdString(m->source))
                             .arg(m->odometry_valid ? "正常" : "异常")
                             .arg(m->odometry_age, 0, 'f', 2)
                             .arg(m->cloud_valid ? "正常" : "异常")
                             .arg(m->cloud_age, 0, 'f', 2));
      });
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/navigation/odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        px_ = m->pose.pose.position.x;
        py_ = m->pose.pose.position.y;
        pz_ = m->pose.pose.position.z;
        const auto& v = m->twist.twist.linear;
        position_->setText(QString("位置：%1, %2, %3 m\n速度：%4 m/s")
                               .arg(px_, 0, 'f', 2)
                               .arg(py_, 0, 'f', 2)
                               .arg(pz_, 0, 'f', 2)
                               .arg(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z), 0, 'f', 2));
      });
  goals_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/navigation/staged_goals", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        if (m->poses.empty()) return;
        has_goal_ = true;
        const auto& p = m->poses.back().pose.position;
        QSignalBlocker bx(x_), by(y_), bz(z_);
        x_->setValue(p.x);
        y_->setValue(p.y);
        z_->setValue(p.z);
        syncGoalPlaneHeight();
      });
  click_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/navigation/ui/clicked_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        if (m->header.frame_id != "odom" || !std::isfinite(m->point.x) ||
            !std::isfinite(m->point.y) || !std::isfinite(m->point.z))
          return;
        if (!buttons_["stage"]->isEnabled()) return;
        x_->setValue(m->point.x);
        y_->setValue(m->point.y);
        z_->setValue(m->point.z);
        setGoal(false);
      });
  duration_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/planning/duration_ms", 10, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
        planner_->setText(QString("最近规划：%1 ms").arg(m->data, 0, 'f', 2));
      });
  timer_->start(50);
  // 两个左侧停靠面板采用标签页，操作台可占满高度而不挤压三维视图。
  QTimer::singleShot(300, this, [this] {
    QMainWindow* main = nullptr;
    for (QWidget* w = parentWidget(); w; w = w->parentWidget())
      if ((main = qobject_cast<QMainWindow*>(w))) break;
    if (!main) return;
    QDockWidget *ours = nullptr, *displays = nullptr;
    for (auto* dock : main->findChildren<QDockWidget*>()) {
      if (dock->isAncestorOf(this)) ours = dock;
      if (dock->windowTitle() == "Displays") displays = dock;
    }
    if (ours && displays) {
      main->tabifyDockWidget(displays, ours);
      ours->raise();
    }
  });
}
void NavigationPanel::updateButtons() {
  bool fresh =
      connected_ && diag_age_.elapsed() < 2000 && have_status_ && status_age_.elapsed() < 2000;
  bool running = phase_ == "EXECUTING" || phase_ == "BRAKING" || phase_ == "WAIT_CONTROL";
  bool edit = have_status_ && status_age_.elapsed() < 2000 && !busy_ && !running;
  bool source_ready = health_age_.isValid() && health_age_.elapsed() < 300 && source_healthy_;
  bool health = diagnostics_["healthy"].toBool();
  bool active = diagnostics_["phase"].toString() == "ACTIVE";
  bool available = diagnostics_["phase"].toString() == "AVAILABLE";
  for (auto* field : {x_, y_, z_}) field->setEnabled(edit);
  for (const auto* name : {"select_point", "stage", "preview", "current_height"})
    buttons_[name]->setEnabled(edit && source_ready && goal_client_ && goal_client_->service_is_ready());
  buttons_["start"]->setEnabled(fresh && edit && source_ready && health && (available || active) && goal_client_ &&
                                goal_client_->service_is_ready() && clients_.count("start") &&
                                clients_["start"]->service_is_ready());
  for (const auto* action : {"pause", "resume", "cancel"}) {
    bool enabled =
        fresh && !busy_ && clients_.count(action) && clients_[action]->service_is_ready();
    if (std::string(action) == "pause") enabled &= phase_ == "EXECUTING" || phase_ == "BRAKING";
    if (std::string(action) == "resume") enabled &= phase_ == "PAUSED";
    buttons_[action]->setEnabled(enabled);
  }
  bool pilot = fresh && !busy_ && diagnostics_["pilot_enabled"].toBool() &&
               diagnostics_["px4_connected"].toBool() && pilot_client_ &&
               pilot_client_->service_is_ready();
  bool armed = diagnostics_["armed"].toBool(), landed = diagnostics_["landed"].toBool();
  bool ready =
      pilot && !diagnostics_["pilot_busy"].toBool() && diagnostics_["pilot_ready"].toBool();
  buttons_["arm"]->setEnabled(ready && landed && !armed);
  buttons_["takeoff"]->setEnabled(ready && landed);
  buttons_["disarm"]->setEnabled(pilot && landed && armed && !diagnostics_["pilot_busy"].toBool());
  buttons_["hold"]->setEnabled(pilot && armed && !landed &&
                               diagnostics_["position_healthy"].toBool());
  buttons_["land"]->setEnabled(pilot && armed && !landed);
  buttons_["reset_control_fault"]->setEnabled(
      fresh && !busy_ && diagnostics_["reset_ready"].toBool() &&
      clients_.count("reset_control_fault") && clients_["reset_control_fault"]->service_is_ready());
  if (!have_status_ || status_age_.elapsed() >= 2000) {
    mission_->setText("导航：连接中断或等待连接");
  }
  if (!fresh) px4_->setText("PX4：状态过期，开始飞行已禁用");
}
void NavigationPanel::finish(const QString& text, bool ok) {
  busy_ = false;
  cancel_ = {};
  result_->setText(text);
  result_->setStyleSheet(ok ? "color:#197756" : "color:#b34032");
  updateButtons();
}
void NavigationPanel::setGoal(bool preview, bool start_after) {
  if (busy_ || !goal_client_ || !goal_client_->service_is_ready()) return;
  auto q = std::make_shared<Goal::Request>();
  q->goal.x = x_->value();
  q->goal.y = y_->value();
  q->goal.z = z_->value();
  q->preview = preview;
  busy_ = true;
  operation_age_.restart();
  auto ticket = ++serial_;
  result_->setText(preview ? "正在预览局部规划…" : "正在设置目标…");
  updateButtons();
  auto future = goal_client_->async_send_request(
      q, [this, ticket, start_after](rclcpp::Client<Goal>::SharedFuture f) {
        if (ticket != serial_) return;
        auto r = f.get();
        finish(QString::fromStdString(r->message), r->success);
        if (r->success && start_after) trigger("start");
      });
  cancel_ = [client = goal_client_, id = future.request_id] { client->remove_pending_request(id); };
}
void NavigationPanel::trigger(const std::string& action) {
  auto client = clients_.at(action);
  if (busy_ || !client->service_is_ready()) return;
  busy_ = true;
  operation_age_.restart();
  auto ticket = ++serial_;
  updateButtons();
  auto f = client->async_send_request(std::make_shared<Trigger::Request>(),
                                      [this, ticket](rclcpp::Client<Trigger>::SharedFuture future) {
                                        if (ticket != serial_) return;
                                        auto r = future.get();
                                        finish(QString::fromStdString(r->message), r->success);
                                      });
  cancel_ = [client, id = f.request_id] { client->remove_pending_request(id); };
}
void NavigationPanel::pilotCommand(const std::string& action) {
  if (busy_ || !pilot_client_ || !pilot_client_->service_is_ready()) return;
  auto q = std::make_shared<Pilot::Request>();
  q->command = action;
  q->height = height_->value();
  busy_ = true;
  operation_age_.restart();
  auto ticket = ++serial_;
  updateButtons();
  auto f = pilot_client_->async_send_request(
      q, [this, ticket](rclcpp::Client<Pilot>::SharedFuture future) {
        if (ticket != serial_) return;
        auto r = future.get();
        finish(QString::fromStdString(r->message), r->success);
      });
  cancel_ = [client = pilot_client_, id = f.request_id] { client->remove_pending_request(id); };
}
void NavigationPanel::syncGoalPlaneHeight() {
  if (!getDisplayContext()) return;
  auto* tools=getDisplayContext()->getToolManager();
  for (int i=0;i<tools->numTools();++i)
    if (tools->getTool(i)->getClassId()=="uav_rviz_plugins/GoalPlaneTool")
      tools->getTool(i)->getPropertyContainer()->subProp("Height")->setValue(z_->value());
}
void NavigationPanel::view(const std::string& kind) {
  if (!getDisplayContext()) return;
  auto* views = getDisplayContext()->getViewManager();
  views->setCurrentViewControllerType("rviz_default_plugins/Orbit");
  auto* v = views->getCurrent();
  v->subProp("Target Frame")->setValue(kind == "follow" ? "base_link" : "odom");
  auto* f = v->subProp("Focal Point");
  f->subProp("X")->setValue(kind == "follow" ? 0 : px_);
  f->subProp("Y")->setValue(kind == "follow" ? 0 : py_);
  f->subProp("Z")->setValue(kind == "follow" ? 0 : pz_);
  v->subProp("Distance")->setValue(kind == "follow" ? 5.0 : 17.0);
  v->subProp("Pitch")->setValue(kind == "top" ? 1.56 : 0.75);
  v->subProp("Yaw")->setValue(.85);
}
}  // namespace uav_rviz_plugins
PLUGINLIB_EXPORT_CLASS(uav_rviz_plugins::NavigationPanel, rviz_common::Panel)
