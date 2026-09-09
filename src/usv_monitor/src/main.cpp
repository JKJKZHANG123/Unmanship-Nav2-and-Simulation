#include <algorithm>
#include <atomic>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <QStringList>

#include <geographic_msgs/msg/geo_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

std::atomic_bool shutdown_requested{false};

void request_shutdown(int)
{
  shutdown_requested.store(true);
}

QString now_text()
{
  return QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
}

double normalize_degrees(double value)
{
  value = std::fmod(value, 360.0);
  if (value < 0.0) {
    value += 360.0;
  }
  return value;
}

QString fix_name(int8_t status)
{
  switch (status) {
    case -1: return "NO_FIX";
    case 0: return "FIX";
    case 1: return "SBAS_FIX";
    case 2: return "GBAS_FIX/RTK";
    default: return QString("STATUS_%1").arg(status);
  }
}

QString number(double value, int precision = 7)
{
  return std::isfinite(value) ? QString::number(value, 'f', precision) : "--";
}

enum class ParameterKind { Bool, Integer, Double, String };

struct ParameterSpec
{
  QString node;
  QString name;
  ParameterKind kind;
  QString initial;
  QString description;
};

}  // namespace

class MainWindow final : public QMainWindow
{
public:
  MainWindow()
  : QMainWindow(), node_(std::make_shared<rclcpp::Node>("usv_monitor"))
  {
    setWindowTitle("USV 实船数据监控台");
    resize(1380, 860);
    initialize_parameter_specs();
    build_ui();
    create_subscriptions();

    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, [this]() {
      // rclcpp is intentionally driven from the Qt GUI thread.  The signal
      // handler only flips an atomic flag; shutdown itself happens here so
      // no callback can run after the ROS context has been invalidated.
      if (shutdown_requested.load()) {
        QCoreApplication::quit();
        return;
      }
      if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
      }
    });
    spin_timer_->start(20);

    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() {
      refresh_topic_table();
      refresh_age_labels();
    });
    refresh_timer_->start(500);
    append_event("监控程序已启动；原始串口数据通过 /gps/mavlink_raw 镜像显示");
  }

  ~MainWindow() override
  {
    if (spin_timer_) {
      spin_timer_->stop();
    }
    if (refresh_timer_) {
      refresh_timer_->stop();
    }
    node_.reset();
  }

private:
  struct TopicState {
    QString type;
    qint64 last_ms{0};
    quint64 count{0};
    double rate_hz{0.0};
    qint64 previous_ms{0};
  };

  void build_ui()
  {
    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);

    auto *position_box = new QGroupBox("RTK 与船体状态", central);
    auto *position_grid = new QGridLayout(position_box);
    add_value(position_grid, 0, 0, "RTK状态", rtk_status_);
    add_value(position_grid, 0, 2, "纬度", latitude_);
    add_value(position_grid, 0, 4, "经度", longitude_);
    add_value(position_grid, 0, 6, "高度(m)", altitude_);
    add_value(position_grid, 1, 0, "卫星数", satellites_);
    add_value(position_grid, 1, 2, "位置更新时间", position_age_);
    add_value(position_grid, 1, 4, "ENU朝向(°)", enu_heading_);
    add_value(position_grid, 1, 6, "罗盘航向(°)", compass_heading_);
    add_value(position_grid, 2, 0, "航向来源", heading_source_);
    add_value(position_grid, 2, 2, "目标纬度", target_latitude_);
    add_value(position_grid, 2, 4, "目标经度", target_longitude_);
    add_value(position_grid, 2, 6, "目标状态", target_status_);
    root->addWidget(position_box);

    auto *pipeline_box = new QGroupBox("规划与感知", central);
    auto *pipeline_grid = new QGridLayout(pipeline_box);
    add_value(pipeline_grid, 0, 0, "局部路径点数", local_plan_);
    add_value(pipeline_grid, 0, 2, "经纬度路径点数", geo_plan_);
    add_value(pipeline_grid, 0, 4, "点云尺寸", cloud_);
    add_value(pipeline_grid, 0, 6, "IMU状态", imu_);
    add_value(pipeline_grid, 1, 0, "代价地图", costmap_);
    add_value(pipeline_grid, 1, 2, "速度输出", cmd_vel_);
    add_value(pipeline_grid, 1, 4, "任务上传", mission_state_);
    add_value(pipeline_grid, 1, 6, "原始串口累计字节", raw_bytes_);
    add_value(pipeline_grid, 2, 0, "LIO位置(m)", lio_position_);
    add_value(pipeline_grid, 2, 2, "LIO航向(°)", lio_heading_);
    add_value(pipeline_grid, 2, 4, "导航状态", nav_mode_);
    add_value(pipeline_grid, 2, 6, "安全停车", safety_stop_);
    root->addWidget(pipeline_box);

    topic_table_ = new QTableWidget(0, 5, central);
    topic_table_->setHorizontalHeaderLabels(
      {"话题", "类型", "消息数", "频率(Hz)", "距今(ms)"});
    topic_table_->horizontalHeader()->setStretchLastSection(true);
    topic_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    topic_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    topic_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    topic_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    root->addWidget(topic_table_, 2);
    connect(topic_table_, &QTableWidget::itemSelectionChanged, this, [this]() {
      const auto rows = topic_table_->selectionModel()->selectedRows();
      if (rows.empty()) {
        selected_topic_->setText("未选择话题");
        detail_view_->setPlainText("在上方表格选择话题后，这里显示该话题最近一条已解析数据。\n"
          "高频点云只显示元数据，避免阻塞 Qt 界面。");
        return;
      }
      const auto topic = topic_table_->item(rows.front().row(), 0)->text();
      selected_topic_->setText(topic);
      const auto it = latest_data_.find(topic);
      detail_view_->setPlainText(it == latest_data_.end() ?
        "该话题尚未收到可解析的消息；动态发现的话题目前只统计频率。" : it->second);
    });

    build_parameter_panel(root, central);

    auto *detail_box = new QGroupBox("话题数据详情（选择上方话题查看）", central);
    auto *detail_layout = new QVBoxLayout(detail_box);
    selected_topic_ = new QLabel("未选择话题", detail_box);
    detail_view_ = make_view();
    detail_view_->setPlaceholderText("选择话题后显示最近一条数据");
    detail_layout->addWidget(selected_topic_);
    detail_layout->addWidget(detail_view_);
    root->addWidget(detail_box, 1);

    auto *bottom_box = new QGroupBox("实时数据面板", central);
    auto *bottom_layout = new QGridLayout(bottom_box);
    raw_view_ = make_view();
    status_view_ = make_view();
    event_view_ = make_view();
    raw_view_->setPlaceholderText("等待 /gps/mavlink_raw ...");
    status_view_->setPlaceholderText("等待 /gps/status ...");
    event_view_->setPlaceholderText("程序事件和错误 ...");
    bottom_layout->addWidget(new QLabel("RTK串口原始镜像", bottom_box), 0, 0);
    bottom_layout->addWidget(new QLabel("桥接状态JSON", bottom_box), 0, 1);
    bottom_layout->addWidget(new QLabel("事件日志", bottom_box), 0, 2);
    bottom_layout->addWidget(raw_view_, 1, 0);
    bottom_layout->addWidget(status_view_, 1, 1);
    bottom_layout->addWidget(event_view_, 1, 2);
    auto *clear_raw = new QPushButton("清空原始数据", bottom_box);
    auto *clear_event = new QPushButton("清空事件", bottom_box);
    bottom_layout->addWidget(clear_raw, 2, 0);
    bottom_layout->addWidget(clear_event, 2, 2);
    connect(clear_raw, &QPushButton::clicked, raw_view_, &QPlainTextEdit::clear);
    connect(clear_event, &QPushButton::clicked, event_view_, &QPlainTextEdit::clear);
    root->addWidget(bottom_box, 2);

    setCentralWidget(central);
  }

  static QPlainTextEdit *make_view()
  {
    auto *view = new QPlainTextEdit();
    view->setReadOnly(true);
    view->setMaximumBlockCount(500);
    view->setStyleSheet("font-family: monospace; font-size: 11px;");
    return view;
  }

  static void add_value(QGridLayout *grid, int row, int column,
                        const QString &name, QLabel *&value)
  {
    auto *label = new QLabel(name + ":");
    value = new QLabel("--");
    value->setTextInteractionFlags(Qt::TextSelectableByMouse);
    value->setMinimumWidth(125);
    grid->addWidget(label, row, column, 1, 1);
    grid->addWidget(value, row, column + 1, 1, 1);
  }

  void initialize_parameter_specs()
  {
    // Only expose parameters that are useful during a real-boat bench test.
    // Changes are sent through the target node's ROS parameter service and do
    // not restart nodes or open any actuator/control interface.
    specs_ = {
      {"/obstacle_filter", "self_mask_mode", ParameterKind::String, "rectangle",
        "rectangle 或 wamv_geometry"},
      {"/obstacle_filter", "self_x_min", ParameterKind::Double, "-0.417698",
        "自船屏蔽后边界（base_link，m）"},
      {"/obstacle_filter", "self_x_max", ParameterKind::Double, "0.792302",
        "自船屏蔽前边界（base_link，m）"},
      {"/obstacle_filter", "self_y_min", ParameterKind::Double, "-0.324655",
        "自船屏蔽右舷边界（base_link，m）"},
      {"/obstacle_filter", "self_y_max", ParameterKind::Double, "0.295345",
        "自船屏蔽左舷边界（base_link，m）"},
      {"/obstacle_filter", "raw_z_min", ParameterKind::Double, "0.50",
        "原始点云最低高度（m）"},
      {"/obstacle_filter", "raw_z_max", ParameterKind::Double, "4.0",
        "原始点云最高高度（m）"},
      {"/obstacle_filter", "structure_z_min", ParameterKind::Double, "0.10",
        "结构障碍最低高度（m）"},
      {"/obstacle_filter", "structure_z_max", ParameterKind::Double, "3.0",
        "结构障碍最高高度（m）"},
      {"/obstacle_filter", "max_range", ParameterKind::Double, "35.0",
        "局部障碍最大距离（m）"},
      {"/obstacle_filter", "voxel_size", ParameterKind::Double, "0.10",
        "点云体素尺寸（m）"},
      {"/obstacle_filter", "enable_outlier", ParameterKind::Bool, "true",
        "是否启用离群点过滤"},
      {"/obstacle_filter", "outlier_radius", ParameterKind::Double, "1.0",
        "离群点邻域半径（m）"},
      {"/obstacle_filter", "outlier_min_neighbors", ParameterKind::Integer, "1",
        "离群点最少邻居数"},
      {"/mavlink_rtk_bridge", "heading_offset_deg", ParameterKind::Double, "0.0",
        "双天线航向安装偏角（度）"},
      {"/mavlink_rtk_bridge", "stale_timeout_s", ParameterKind::Double, "2.0",
        "RTK数据过期时间（秒）"},
      {"/mavlink_rtk_bridge", "mission_upload_enabled", ParameterKind::Bool, "false",
        "是否允许上传路径任务（实船谨慎开启）"},
      {"/mavlink_rtk_bridge", "mission_acceptance_radius_m", ParameterKind::Double, "1.0",
        "飞控任务点接受半径（m）"},
      {"/target_to_goal", "max_target_distance_m", ParameterKind::Double, "5000.0",
        "目标点最大允许距离（m）"},
      {"/geodetic_goal_planner", "replan_period_s", ParameterKind::Double, "1.0",
        "经纬度目标重规划周期（秒）"},
      {"/geodetic_goal_planner", "max_retries", ParameterKind::Integer, "0",
        "规划服务最大重试次数"},
      {"/local_costmap/local_costmap", "inflation_radius", ParameterKind::Double, "2.0",
        "局部代价地图膨胀半径（m）"},
      {"/local_costmap/local_costmap", "obstacle_max_range", ParameterKind::Double, "35.0",
        "局部障碍层最大距离（m）"},
      {"/local_costmap/local_costmap", "raytrace_max_range", ParameterKind::Double, "40.0",
        "局部清除射线最大距离（m）"},
      {"/local_costmap/local_costmap", "footprint", ParameterKind::String,
        "[[0.792302, 0.295345], [0.792302, -0.324655], [-0.417698, -0.324655], [-0.417698, 0.295345]]",
        "船体二维 footprint；Nav2 参数格式为字符串"},
    };
  }

  void build_parameter_panel(QVBoxLayout *root, QWidget *parent)
  {
    auto *box = new QGroupBox("核心参数在线读写（ROS 参数服务）", parent);
    auto *grid = new QGridLayout(box);
    param_node_combo_ = new QComboBox(box);
    param_name_combo_ = new QComboBox(box);
    param_value_edit_ = new QLineEdit(box);
    param_hint_ = new QLabel("--", box);
    param_hint_->setWordWrap(true);
    param_result_ = new QLabel("未执行读写", box);
    auto *read = new QPushButton("读取参数", box);
    auto *write = new QPushButton("写入参数", box);
    param_node_combo_->setMinimumWidth(250);
    param_name_combo_->setMinimumWidth(250);
    param_value_edit_->setMinimumWidth(380);
    for (const auto &spec : specs_) {
      if (param_node_combo_->findText(spec.node) < 0) {
        param_node_combo_->addItem(spec.node);
      }
    }
    grid->addWidget(new QLabel("目标节点", box), 0, 0);
    grid->addWidget(param_node_combo_, 0, 1);
    grid->addWidget(new QLabel("参数", box), 0, 2);
    grid->addWidget(param_name_combo_, 0, 3);
    grid->addWidget(new QLabel("值", box), 1, 0);
    grid->addWidget(param_value_edit_, 1, 1, 1, 3);
    grid->addWidget(new QLabel("说明", box), 2, 0);
    grid->addWidget(param_hint_, 2, 1, 1, 3);
    grid->addWidget(read, 3, 0, 1, 2);
    grid->addWidget(write, 3, 2, 1, 2);
    grid->addWidget(param_result_, 4, 0, 1, 4);
    root->addWidget(box);

    connect(param_node_combo_, &QComboBox::currentTextChanged, this,
      [this](const QString &) { refresh_parameter_names(); });
    connect(param_name_combo_, &QComboBox::currentTextChanged, this,
      [this](const QString &) { refresh_parameter_editor(); });
    connect(read, &QPushButton::clicked, this, [this]() { read_selected_parameter(); });
    connect(write, &QPushButton::clicked, this, [this]() { write_selected_parameter(); });
    refresh_parameter_names();
  }

  const ParameterSpec *selected_parameter() const
  {
    const auto node = param_node_combo_->currentText();
    const auto name = param_name_combo_->currentText();
    for (const auto &spec : specs_) {
      if (spec.node == node && spec.name == name) {
        return &spec;
      }
    }
    return nullptr;
  }

  void refresh_parameter_names()
  {
    if (!param_name_combo_) return;
    const auto node = param_node_combo_->currentText();
    param_name_combo_->blockSignals(true);
    param_name_combo_->clear();
    for (const auto &spec : specs_) {
      if (spec.node == node) param_name_combo_->addItem(spec.name);
    }
    param_name_combo_->blockSignals(false);
    refresh_parameter_editor();
  }

  void refresh_parameter_editor()
  {
    const auto *spec = selected_parameter();
    if (!spec) return;
    param_value_edit_->setText(spec->initial);
    param_hint_->setText(QString("类型：%1；%2")
      .arg(parameter_kind_name(spec->kind), spec->description));
    param_result_->setText("未执行读写");
  }

  static QString parameter_kind_name(ParameterKind kind)
  {
    switch (kind) {
      case ParameterKind::Bool: return "bool（true/false）";
      case ParameterKind::Integer: return "整数";
      case ParameterKind::Double: return "浮点数";
      case ParameterKind::String: return "字符串";
    }
    return "未知";
  }

  QString service_node_name() const
  {
    auto name = param_node_combo_->currentText();
    if (!name.startsWith('/')) name.prepend('/');
    return name;
  }

  void read_selected_parameter()
  {
    const auto *spec = selected_parameter();
    if (!spec) return;
    auto client = node_->create_client<rcl_interfaces::srv::GetParameters>(
      service_node_name().toStdString() + "/get_parameters");
    if (!client->service_is_ready()) {
      param_result_->setText("读取失败：目标节点参数服务尚未就绪");
      append_event("参数读取失败：" + service_node_name());
      return;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(spec->name.toStdString());
    param_result_->setText("正在读取...");
    client->async_send_request(request,
      [this, kind = spec->kind, name = spec->name](
        std::shared_future<rcl_interfaces::srv::GetParameters::Response::SharedPtr> future) {
        const auto response = future.get();
        if (!response || response->values.empty()) {
          param_result_->setText("读取失败：目标节点未返回参数");
          return;
        }
        param_value_edit_->setText(parameter_value_text(response->values.front(), kind));
        param_result_->setText("读取成功：" + name);
      });
  }

  void write_selected_parameter()
  {
    const auto *spec = selected_parameter();
    if (!spec) return;
    rclcpp::Parameter parameter;
    const auto value = param_value_edit_->text();
    try {
      bool ok = false;
      switch (spec->kind) {
        case ParameterKind::Bool: {
          const auto v = value.trimmed().toLower();
          if (v != "true" && v != "false" && v != "1" && v != "0")
            throw std::runtime_error("bool must be true/false");
          parameter = rclcpp::Parameter(spec->name.toStdString(), v == "true" || v == "1");
          break;
        }
        case ParameterKind::Integer: {
          const auto v = value.toLongLong(&ok);
          if (!ok) throw std::runtime_error("invalid integer");
          parameter = rclcpp::Parameter(spec->name.toStdString(), static_cast<int64_t>(v));
          break;
        }
        case ParameterKind::Double: {
          const auto v = value.toDouble(&ok);
          if (!ok || !std::isfinite(v)) throw std::runtime_error("invalid double");
          parameter = rclcpp::Parameter(spec->name.toStdString(), v);
          break;
        }
        case ParameterKind::String:
          parameter = rclcpp::Parameter(spec->name.toStdString(), value.toStdString());
          break;
      }
    } catch (const std::exception &e) {
      param_result_->setText("输入错误：" + QString::fromUtf8(e.what()));
      return;
    }
    auto client = node_->create_client<rcl_interfaces::srv::SetParameters>(
      service_node_name().toStdString() + "/set_parameters");
    if (!client->service_is_ready()) {
      param_result_->setText("写入失败：目标节点参数服务尚未就绪");
      append_event("参数写入失败：" + service_node_name());
      return;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(parameter.to_parameter_msg());
    param_result_->setText("正在写入...");
    client->async_send_request(request,
      [this, name = spec->name](
        std::shared_future<rcl_interfaces::srv::SetParameters::Response::SharedPtr> future) {
        const auto response = future.get();
        if (!response || response->results.empty() || !response->results.front().successful) {
          const auto reason = response->results.empty() ? "无返回结果" :
            response->results.front().reason;
          param_result_->setText("写入失败：" + QString::fromStdString(reason));
          append_event("参数写入失败：" + name);
          return;
        }
        param_result_->setText("写入成功：" + name);
        append_event("参数写入成功：" + name);
      });
  }

  static QString parameter_value_text(
    const rcl_interfaces::msg::ParameterValue &value, ParameterKind kind)
  {
    switch (kind) {
      case ParameterKind::Bool:
        return value.bool_value ? "true" : "false";
      case ParameterKind::Integer:
        return QString::number(value.integer_value);
      case ParameterKind::Double:
        return QString::number(value.double_value, 'g', 15);
      case ParameterKind::String:
        return QString::fromStdString(value.string_value);
    }
    return "--";
  }

  void record_data(const QString &topic, const QString &text)
  {
    latest_data_[topic] = now_text() + " " + text;
    if (selected_topic_ && selected_topic_->text() == topic) {
      detail_view_->setPlainText(latest_data_[topic]);
    }
  }

  void create_subscriptions()
  {
    using sensor_msgs::msg::NavSatFix;
    const auto qos = rclcpp::QoS(20).reliable();

    fix_sub_ = node_->create_subscription<NavSatFix>(
      "/gps/fix", qos, [this](NavSatFix::ConstSharedPtr msg) {
        mark("/gps/fix", "sensor_msgs/NavSatFix");
        rtk_status_->setText(fix_name(msg->status.status));
        latitude_->setText(number(msg->latitude));
        longitude_->setText(number(msg->longitude));
        altitude_->setText(number(msg->altitude, 3));
        satellites_->setText("--");
        position_age_->setText("0");
        record_data("/gps/fix", QString("lat=%1 lon=%2 alt=%3 status=%4")
          .arg(number(msg->latitude)).arg(number(msg->longitude))
          .arg(number(msg->altitude, 3)).arg(fix_name(msg->status.status)));
      });

    heading_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/gps/heading", qos, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        mark("/gps/heading", "std_msgs/Float64");
        const double enu = normalize_degrees(msg->data * 180.0 / M_PI);
        const double compass = normalize_degrees(90.0 - enu);
        enu_heading_->setText(number(enu, 2));
        compass_heading_->setText(number(compass, 2));
        record_data("/gps/heading", QString("ENU yaw=%1 deg, compass=%2 deg")
          .arg(number(enu, 2)).arg(number(compass, 2)));
      });

    heading_source_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/gps/heading_source", qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/gps/heading_source", "std_msgs/String");
        heading_source_->setText(QString::fromStdString(msg->data));
        record_data("/gps/heading_source", QString::fromStdString(msg->data));
      });

    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/gps/status", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/gps/status", "std_msgs/String");
        status_view_->setPlainText(QString::fromStdString(msg->data));
        const auto document = QJsonDocument::fromJson(
          QByteArray::fromStdString(msg->data));
        record_data("/gps/status", QString::fromStdString(msg->data));
        if (document.isObject()) {
          const auto object = document.object();
          const auto mission = object.value("mission_state").toString();
          const auto points = object.value("mission_points").toInt();
          const auto fix = object.value("fix").toObject();
          if (fix.contains("satellites_visible")) {
            satellites_->setText(
              QString::number(fix.value("satellites_visible").toInt()));
          }
          mission_state_->setText(
            mission.isEmpty() ? "--" : QString("%1 (%2点)").arg(mission).arg(points));
          raw_bytes_->setText(QString::number(
            object.value("raw_bytes_rx").toVariant().toLongLong()));
        }
      });

    raw_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/gps/mavlink_raw", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/gps/mavlink_raw", "std_msgs/String");
        const QString raw = QString::fromStdString(msg->data);
        raw_view_->appendPlainText(now_text() + " " + raw);
        record_data("/gps/mavlink_raw", raw);
        // The bridge prefixes each diagnostic packet with total=<n>.  Keep a
        // useful cumulative byte counter even when the JSON status topic is slow.
        const auto total_pos = raw.indexOf("total=");
        if (total_pos >= 0) {
          int end = total_pos + 6;
          while (end < raw.size() && raw[end].isDigit()) {
            ++end;
          }
          bool ok = false;
          const auto total = raw.mid(total_pos + 6, end - total_pos - 6).toLongLong(&ok);
          if (ok) {
            raw_bytes_->setText(QString::number(total));
          }
        }
      });

    target_sub_ = node_->create_subscription<NavSatFix>(
      "/target/fix", qos, [this](NavSatFix::ConstSharedPtr msg) {
        mark("/target/fix", "sensor_msgs/NavSatFix");
        target_latitude_->setText(number(msg->latitude));
        target_longitude_->setText(number(msg->longitude));
        target_status_->setText(fix_name(msg->status.status));
        record_data("/target/fix", QString("target lat=%1 lon=%2 status=%3")
          .arg(number(msg->latitude)).arg(number(msg->longitude))
          .arg(fix_name(msg->status.status)));
      });

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", qos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        mark("/goal_pose", "geometry_msgs/PoseStamped");
        record_data("/goal_pose", QString("frame=%1 x=%2 y=%3 z=%4")
          .arg(QString::fromStdString(msg->header.frame_id))
          .arg(msg->pose.position.x, 0, 'f', 3)
          .arg(msg->pose.position.y, 0, 'f', 3)
          .arg(msg->pose.position.z, 0, 'f', 3));
      });

    plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/plan", qos, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        mark("/plan", "nav_msgs/Path");
        local_plan_->setText(QString::number(msg->poses.size()));
        record_data("/plan", QString("frame=%1 points=%2")
          .arg(QString::fromStdString(msg->header.frame_id)).arg(msg->poses.size()));
      });

    geo_plan_sub_ = node_->create_subscription<geographic_msgs::msg::GeoPath>(
      "/plan_geodetic", qos,
      [this](geographic_msgs::msg::GeoPath::ConstSharedPtr msg) {
        mark("/plan_geodetic", "geographic_msgs/GeoPath");
        geo_plan_->setText(QString::number(msg->poses.size()));
        QString geo = QString("frame=%1 points=%2")
          .arg(QString::fromStdString(msg->header.frame_id)).arg(msg->poses.size());
        if (!msg->poses.empty()) {
          const auto &first = msg->poses.front().pose.position;
          const auto &last = msg->poses.back().pose.position;
          geo += QString("\nfirst=(%1,%2) last=(%3,%4)")
            .arg(first.latitude, 0, 'f', 8).arg(first.longitude, 0, 'f', 8)
            .arg(last.latitude, 0, 'f', 8).arg(last.longitude, 0, 'f', 8);
        }
        record_data("/plan_geodetic", geo);
      });

    geo_json_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/plan_geodetic_json", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/plan_geodetic_json", "std_msgs/String");
        record_data("/plan_geodetic_json", QString::fromStdString(msg->data));
      });

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/unilidar/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        mark("/unilidar/imu", "sensor_msgs/Imu");
        imu_->setText(QString("%1 Hz").arg(rate_for("/unilidar/imu"), 0, 'f', 1));
        record_data("/unilidar/imu", QString("frame=%1 accel=(%2,%3,%4) gyro=(%5,%6,%7)")
          .arg(QString::fromStdString(msg->header.frame_id))
          .arg(msg->linear_acceleration.x, 0, 'f', 3)
          .arg(msg->linear_acceleration.y, 0, 'f', 3)
          .arg(msg->linear_acceleration.z, 0, 'f', 3)
          .arg(msg->angular_velocity.x, 0, 'f', 3)
          .arg(msg->angular_velocity.y, 0, 'f', 3)
          .arg(msg->angular_velocity.z, 0, 'f', 3));
      });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/unilidar/cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        mark("/unilidar/cloud", "sensor_msgs/PointCloud2");
        cloud_->setText(QString("%1 x %2").arg(msg->width).arg(msg->height));
        record_data("/unilidar/cloud", QString("frame=%1 width=%2 height=%3 fields=%4 point_step=%5")
          .arg(QString::fromStdString(msg->header.frame_id)).arg(msg->width).arg(msg->height)
          .arg(msg->fields.size()).arg(msg->point_step));
      });

    const auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
    costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        mark("/local_costmap/costmap", "nav_msgs/OccupancyGrid");
        costmap_->setText(QString("%1 x %2").arg(msg->info.width).arg(msg->info.height));
        record_data("/local_costmap/costmap", QString("frame=%1 size=%2x%3 resolution=%4 origin=(%5,%6)")
          .arg(QString::fromStdString(msg->header.frame_id)).arg(msg->info.width).arg(msg->info.height)
          .arg(msg->info.resolution, 0, 'f', 3).arg(msg->info.origin.position.x, 0, 'f', 2)
          .arg(msg->info.origin.position.y, 0, 'f', 2));
      });

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", qos,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        mark("/cmd_vel_smoothed", "geometry_msgs/Twist");
        cmd_vel_->setText(QString("lin=%1 ang=%2")
          .arg(msg->linear.x, 0, 'f', 2).arg(msg->angular.z, 0, 'f', 2));
        record_data("/cmd_vel_smoothed", QString("linear=(%1,%2,%3) angular=(%4,%5,%6)")
          .arg(msg->linear.x, 0, 'f', 3).arg(msg->linear.y, 0, 'f', 3).arg(msg->linear.z, 0, 'f', 3)
          .arg(msg->angular.x, 0, 'f', 3).arg(msg->angular.y, 0, 'f', 3).arg(msg->angular.z, 0, 'f', 3));
      });

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        mark("/aft_mapped_to_init", "nav_msgs/Odometry");
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        const double yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        lio_position_->setText(QString("%1, %2, %3")
          .arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2).arg(p.z, 0, 'f', 2));
        lio_heading_->setText(number(normalize_degrees(yaw * 180.0 / M_PI), 2));
        record_data("/aft_mapped_to_init", QString("frame=%1 position=(%2,%3,%4) yaw=%5 deg")
          .arg(QString::fromStdString(msg->header.frame_id)).arg(p.x, 0, 'f', 3)
          .arg(p.y, 0, 'f', 3).arg(p.z, 0, 'f', 3)
          .arg(normalize_degrees(yaw * 180.0 / M_PI), 0, 'f', 2));
      });

    nav_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/nav_mode", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/nav_mode", "std_msgs/String");
        nav_mode_->setText(QString::fromStdString(msg->data));
        record_data("/nav_mode", QString::fromStdString(msg->data));
      });

    goal_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/usv/goal_status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mark("/usv/goal_status", "std_msgs/String");
        append_event("目标状态: " + QString::fromStdString(msg->data));
      });

    safety_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/usv/safety_stop", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        mark("/usv/safety_stop", "std_msgs/Bool");
        safety_stop_->setText(msg->data ? "触发" : "正常");
        safety_stop_->setStyleSheet(msg->data ? "color: red; font-weight: bold;" : "color: green;");
        record_data("/usv/safety_stop", msg->data ? "true（触发）" : "false（正常）");
      });
  }

  void mark(const QString &topic, const QString &type)
  {
    auto &state = topics_[topic];
    state.type = type;
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (state.last_ms > 0 && now > state.last_ms) {
      const double instant = 1000.0 / static_cast<double>(now - state.last_ms);
      state.rate_hz = state.rate_hz == 0.0 ? instant :
        0.8 * state.rate_hz + 0.2 * instant;
    }
    state.previous_ms = state.last_ms;
    state.last_ms = now;
    ++state.count;
  }

  double rate_for(const QString &topic) const
  {
    const auto it = topics_.find(topic);
    return it == topics_.end() ? 0.0 : it->second.rate_hz;
  }

  void refresh_topic_table()
  {
    const auto discovered = node_->get_topic_names_and_types();
    for (const auto &entry : discovered) {
      auto &state = topics_[QString::fromStdString(entry.first)];
      if (state.type.isEmpty() && !entry.second.empty()) {
        state.type = QString::fromStdString(entry.second.front());
      }
    }

    QStringList names;
    for (const auto &entry : topics_) {
      names << entry.first;
    }
    names.sort();
    topic_table_->setRowCount(names.size());
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (int row = 0; row < names.size(); ++row) {
      const auto &name = names[row];
      const auto &state = topics_.at(name);
      topic_table_->setItem(row, 0, new QTableWidgetItem(name));
      topic_table_->setItem(row, 1, new QTableWidgetItem(state.type));
      topic_table_->setItem(row, 2,
        new QTableWidgetItem(QString::number(state.count)));
      topic_table_->setItem(row, 3,
        new QTableWidgetItem(QString::number(state.rate_hz, 'f', 1)));
      const QString age = state.last_ms == 0 ? "--" :
        QString::number(std::max<qint64>(0, now - state.last_ms));
      topic_table_->setItem(row, 4, new QTableWidgetItem(age));
    }
  }

  void refresh_age_labels()
  {
    const auto it = topics_.find("/gps/fix");
    if (it != topics_.end() && it->second.last_ms > 0) {
      position_age_->setText(QString::number(
        QDateTime::currentMSecsSinceEpoch() - it->second.last_ms));
    }
  }

  void append_event(const QString &text)
  {
    event_view_->appendPlainText(now_text() + " " + text);
  }

  std::shared_ptr<rclcpp::Node> node_;
  QTimer *spin_timer_{nullptr};
  QTimer *refresh_timer_{nullptr};
  QTableWidget *topic_table_{nullptr};
  QPlainTextEdit *raw_view_{nullptr};
  QPlainTextEdit *status_view_{nullptr};
  QPlainTextEdit *event_view_{nullptr};
  QPlainTextEdit *detail_view_{nullptr};
  QLabel *selected_topic_{nullptr};

  QComboBox *param_node_combo_{nullptr};
  QComboBox *param_name_combo_{nullptr};
  QLineEdit *param_value_edit_{nullptr};
  QLabel *param_hint_{nullptr};
  QLabel *param_result_{nullptr};

  QLabel *rtk_status_{nullptr};
  QLabel *latitude_{nullptr};
  QLabel *longitude_{nullptr};
  QLabel *altitude_{nullptr};
  QLabel *satellites_{nullptr};
  QLabel *position_age_{nullptr};
  QLabel *enu_heading_{nullptr};
  QLabel *compass_heading_{nullptr};
  QLabel *heading_source_{nullptr};
  QLabel *target_latitude_{nullptr};
  QLabel *target_longitude_{nullptr};
  QLabel *target_status_{nullptr};
  QLabel *local_plan_{nullptr};
  QLabel *geo_plan_{nullptr};
  QLabel *cloud_{nullptr};
  QLabel *imu_{nullptr};
  QLabel *costmap_{nullptr};
  QLabel *cmd_vel_{nullptr};
  QLabel *mission_state_{nullptr};
  QLabel *raw_bytes_{nullptr};
  QLabel *lio_position_{nullptr};
  QLabel *lio_heading_{nullptr};
  QLabel *nav_mode_{nullptr};
  QLabel *safety_stop_{nullptr};

  std::map<QString, TopicState> topics_;
  std::map<QString, QString> latest_data_;
  std::vector<ParameterSpec> specs_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heading_source_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPath>::SharedPtr geo_plan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr geo_json_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
};

int main(int argc, char **argv)
{
  // Let Qt own the GUI event loop and translate SIGINT/SIGTERM into a normal
  // Qt shutdown.  rclcpp's default signal handler calls shutdown immediately,
  // which can invalidate a context while the Qt timer is in spin_some().
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  const int result = app.exec();
  rclcpp::shutdown();
  return result;
}
