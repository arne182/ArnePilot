#include <stdio.h>
#include <string.h>
#include <math.h>
#include <map>
#include "common/util.h"
#include "paint.hpp"
#include "sidebar.hpp"

static void draw_background(UIState *s) {
  const NVGcolor color = nvgRGBA(0x0, 0x0, 0x0, 0xff);
  ui_fill_rect(s->vg, {0, 0, sbr_w, s->fb_h}, color);
}

static void draw_settings_button(UIState *s) {
  ui_draw_image(s, settings_btn, "button_settings", 0.65f);
}

static void draw_home_button(UIState *s) {
  ui_draw_image(s, home_btn, "button_home", 1.0f);
}

static void draw_network_strength(UIState *s) {
  static std::map<cereal::DeviceState::NetworkStrength, int> network_strength_map = {
      {cereal::DeviceState::NetworkStrength::UNKNOWN, 1},
      {cereal::DeviceState::NetworkStrength::POOR, 2},
      {cereal::DeviceState::NetworkStrength::MODERATE, 3},
      {cereal::DeviceState::NetworkStrength::GOOD, 4},
      {cereal::DeviceState::NetworkStrength::GREAT, 5}};
  const int img_idx = s->scene.deviceState.getNetworkType() == cereal::DeviceState::NetworkType::NONE ? 0 : network_strength_map[s->scene.deviceState.getNetworkStrength()];
  ui_draw_image(s, {58, 196, 176, 27}, util::string_format("network_%d", img_idx).c_str(), 1.0f);
}

static void draw_ip_addr(UIState *s) {
  const int network_ip_w = 220;
  const int network_ip_x = !s->sidebar_collapsed ? 38 : -(sbr_w);
  const int network_ip_y = 255;

  char network_ip_str[20];
  snprintf(network_ip_str, sizeof(network_ip_str), "%s", s->scene.ipAddr);
  nvgFillColor(s->vg, COLOR_WHITE);
  nvgFontSize(s->vg, 28);
  nvgFontFace(s->vg, "sans-bold");
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgTextBox(s->vg, network_ip_x, network_ip_y, network_ip_w, network_ip_str, NULL);
}

static void draw_battery_text(UIState *s) {
  const int battery_img_w = 96;
  const int battery_img_x = !s->sidebar_collapsed ? 150 : -(sbr_w);
  const int battery_img_y = 303;

  char battery_str[7];
  snprintf(battery_str, sizeof(battery_str), "%d%%%s", s->scene.deviceState.getBatteryPercent(), s->scene.deviceState.getBatteryStatus() == "Charging" ? "+" : "-");
  nvgFillColor(s->vg, COLOR_WHITE);
  nvgFontSize(s->vg, 44*0.8);
  nvgFontFace(s->vg, "sans-regular");
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  nvgTextBox(s->vg, battery_img_x, battery_img_y, battery_img_w, battery_str, NULL);
}

static void draw_metric(UIState *s, const char *label_str, const char *value_str, const int severity, const int y_offset, const char *message_str) {
  NVGcolor status_color;

  if (severity == 0) {
    status_color = COLOR_WHITE;
  } else if (severity == 1) {
    status_color = COLOR_YELLOW;
  } else if (severity > 1) {
    status_color = COLOR_RED;
  }

  const Rect rect = {30, 338 + y_offset, 240, message_str ? strchr(message_str, '\n') ? 124 : 100 : 148};
  ui_draw_rect(s->vg, rect, severity > 0 ? COLOR_WHITE : COLOR_WHITE_ALPHA(85), 2, 20.);

  nvgBeginPath(s->vg);
  nvgRoundedRectVarying(s->vg, rect.x + 6, rect.y + 6, 18, rect.h - 12, 25, 0, 0, 25);
  nvgFillColor(s->vg, status_color);
  nvgFill(s->vg);

  if (!message_str) {
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 78);
    nvgFontFace(s->vg, "sans-bold");
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, rect.x + 50, rect.y + 50, rect.w - 60, value_str, NULL);

    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 48);
    nvgFontFace(s->vg, "sans-regular");
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, rect.x + 50, rect.y + 50 + 66, rect.w - 60, label_str, NULL);
  } else {
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 48);
    nvgFontFace(s->vg, "sans-bold");
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, rect.x + 35, rect.y + (strchr(message_str, '\n') ? 40 : 50), rect.w - 50, message_str, NULL);
  }
}

static void draw_temp_metric(UIState *s) {
  static std::map<cereal::DeviceState::ThermalStatus, const int> temp_severity_map = {
      {cereal::DeviceState::ThermalStatus::GREEN, 0},
      {cereal::DeviceState::ThermalStatus::YELLOW, 1},
      {cereal::DeviceState::ThermalStatus::RED, 2},
      {cereal::DeviceState::ThermalStatus::DANGER, 3}};
  std::string temp_val = std::to_string((int)s->scene.deviceState.getAmbientTempC()) + "°C";
  draw_metric(s, "TEMP", temp_val.c_str(), temp_severity_map[s->scene.deviceState.getThermalStatus()], 0, NULL);
}

static void draw_panda_metric(UIState *s) {
  const int panda_y_offset = 32 + 148;

  int panda_severity = 0;
  std::string panda_message = "VEHICLE\nONLINE";
  if (s->scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
    panda_severity = 2;
    panda_message = "NO\nPANDA";
  }
#ifdef QCOM2
  else if (s->scene.started) {
    panda_severity = s->scene.gpsOK ? 0 : 1;
    panda_message = util::string_format("SAT CNT\n%d", s->scene.satelliteCount);
  }
#endif

  draw_metric(s, NULL, NULL, panda_severity, panda_y_offset, panda_message.c_str());
}

static void draw_connectivity(UIState *s) {
  static std::map<NetStatus, std::pair<const char *, int>> connectivity_map = {
      {NET_ERROR, {"CONNECT\nERROR", 2}},
      {NET_CONNECTED, {"CONNECT\nONLINE", 0}},
      {NET_DISCONNECTED, {"CONNECT\nOFFLINE", 1}},
  };
  auto net_params = connectivity_map[s->scene.athenaStatus];
  draw_metric(s, NULL, NULL, net_params.second, 180 + 158, net_params.first);
}

void ui_draw_sidebar(UIState *s) {
  if (s->sidebar_collapsed) {
    return;
  }
  draw_background(s);
  draw_settings_button(s);
  draw_home_button(s);
  draw_network_strength(s);
  draw_ip_addr(s);
  draw_battery_text(s);
  draw_temp_metric(s);
  draw_panda_metric(s);
  draw_connectivity(s);
}
