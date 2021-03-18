#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/resource.h>

#include <algorithm>

#include "common/util.h"
#include "common/params.h"
#include "common/touch.h"
#include "common/swaglog.h"

#include "ui.hpp"
#include "paint.hpp"
#include "android/sl_sound.hpp"

ExitHandler do_exit;
static void ui_set_brightness(UIState *s, int brightness) {
  static int last_brightness = -1;
  if (last_brightness != brightness && (s->awake || brightness == 0)) {
    if (set_brightness(brightness)) {
      last_brightness = brightness;
    }
  }
}

static void handle_display_state(UIState *s, bool user_input) {
  static int awake_timeout = 0;

  constexpr float accel_samples = 5*UI_FREQ;
  static float accel_prev = 0., gyro_prev = 0.;

  bool should_wake = s->started || s->ignition || user_input;
  if (!should_wake) {
    // tap detection while display is off
    bool accel_trigger = abs(s->accel_sensor - accel_prev) > 0.2;
    bool gyro_trigger = abs(s->gyro_sensor - gyro_prev) > 0.15;
    should_wake = accel_trigger && gyro_trigger;
    gyro_prev = s->gyro_sensor;
    accel_prev = (accel_prev * (accel_samples - 1) + s->accel_sensor) / accel_samples;
  }

  // determine desired state
  bool should_wake = s->awake;
  if (s->scene.dpUiScreenOffReversing && s->scene.isReversing) {
    should_wake = false;
    awake_timeout = 0;
  } else if (user_input || (!s->scene.dpUiScreenOffDriving && (s->ignition || s->started))) {
    should_wake = true;
    awake_timeout = 30*UI_FREQ;
  } else if (awake_timeout > 0){
    --awake_timeout;
    should_wake = true;
  }

  // handle state transition
  if (s->awake != should_wake) {
    s->awake = should_wake;
    int display_mode = s->awake ? HWC_POWER_MODE_NORMAL : HWC_POWER_MODE_OFF;
    LOGW("setting display mode %d", display_mode);
    s->fb->set_power(display_mode);

    if (s->awake) {
      system("service call window 18 i32 1");
    }
  }
}

static bool handle_dp_btn_touch(UIState *s, int touch_x, int touch_y) {
  //dfButton manager  // code below thanks to kumar: https://github.com/arne182/openpilot/commit/71d5aac9f8a3f5942e89634b20cbabf3e19e3e78
  if (s->started && s->active_app != cereal::UiLayoutState::App::SETTINGS) {
    if (s->scene.dpDynamicFollow > 0 && touch_x >= df_btn_x && touch_x <= (df_btn_x + df_btn_w) && touch_y >= df_btn_y && touch_y <= (df_btn_y + df_btn_h)) {
      int val = s->scene.dpDynamicFollow;
      val++;
      if (val >= 5) {
        val = 1;
      }

      char str[2] = {0};
      sprintf(str, "%d", val);
      Params().write_db_value("dp_dynamic_follow", str, 1);

      char time_str[11];
      snprintf(time_str, 11, "%lu", time(NULL));
      Params().write_db_value("dp_last_modified", time_str, 11);
      return true;
    } else if (s->scene.dpAccelProfile > 0 && touch_x >= ap_btn_x && touch_x <= (ap_btn_x + ap_btn_w) && touch_y >= ap_btn_y && touch_y <= (ap_btn_y + ap_btn_h)) {
      int val = s->scene.dpAccelProfile;
      val++;
      if (val >= 4) {
        val = 1;
      }

      char str[2] = {0};
      sprintf(str, "%d", val);
      Params().write_db_value("dp_accel_profile", str, 1);

      char time_str[11];
      snprintf(time_str, 11, "%lu", time(NULL));
      Params().write_db_value("dp_last_modified", time_str, 11);
      return true;
    }
  }
  return false;
}

static void handle_vision_touch(UIState *s, int touch_x, int touch_y) {
  if (s->started && (touch_x >= s->viz_rect.x - bdr_s)
      && (s->active_app != cereal::UiLayoutState::App::SETTINGS)) {
    if (!s->scene.frontview) {
      s->sidebar_collapsed = !s->sidebar_collapsed;
    } else {
      Params().write_db_value("IsDriverViewEnabled", "0", 1);
    }
  }
}

static void handle_sidebar_touch(UIState *s, int touch_x, int touch_y) {
  if (!s->sidebar_collapsed && touch_x <= sbr_w) {
    if (settings_btn.ptInRect(touch_x, touch_y)) {
      s->active_app = cereal::UiLayoutState::App::SETTINGS;
    } else if (home_btn.ptInRect(touch_x, touch_y)) {
      if (s->started) {
        s->active_app = cereal::UiLayoutState::App::NONE;
        s->sidebar_collapsed = true;
      } else {
        s->active_app = cereal::UiLayoutState::App::HOME;
      }
    }
  }
}

static void update_offroad_layout_state(UIState *s, PubMaster *pm) {
  static int timeout = 0;
  static bool prev_collapsed = false;
  static cereal::UiLayoutState::App prev_app = cereal::UiLayoutState::App::NONE;
  if (timeout > 0) {
    timeout--;
  }
  if (prev_collapsed != s->sidebar_collapsed || prev_app != s->active_app || timeout == 0) {
    MessageBuilder msg;
    auto layout = msg.initEvent().initUiLayoutState();
    layout.setActiveApp(s->active_app);
    layout.setSidebarCollapsed(s->sidebar_collapsed);
    pm->send("offroadLayout", msg);
    LOGD("setting active app to %d with sidebar %d", (int)s->active_app, s->sidebar_collapsed);
    prev_collapsed = s->sidebar_collapsed;
    prev_app = s->active_app;
    timeout = 2 * UI_FREQ;
  }
}

int main(int argc, char* argv[]) {
  setpriority(PRIO_PROCESS, 0, -14);
  SLSound sound;

  UIState uistate = {};
  UIState *s = &uistate;
  ui_init(s);
  s->sound = &sound;

  TouchState touch = {0};
  touch_init(&touch);
  handle_display_state(s, true);

  PubMaster *pm = new PubMaster({"offroadLayout"});

  // light sensor scaling and volume params
  const bool LEON = util::read_file("/proc/cmdline").find("letv") != std::string::npos;

  float brightness_b = 0, brightness_m = 0;
  int result = read_param(&brightness_b, "BRIGHTNESS_B", true);
  result += read_param(&brightness_m, "BRIGHTNESS_M", true);
  if (result != 0) {
    brightness_b = LEON ? 10.0 : 5.0;
    brightness_m = LEON ? 2.6 : 1.3;
    write_param_float(brightness_b, "BRIGHTNESS_B", true);
    write_param_float(brightness_m, "BRIGHTNESS_M", true);
  }
  float smooth_brightness = brightness_b;

  const int MIN_VOLUME = LEON ? 12 : 9;
  const int MAX_VOLUME = LEON ? 15 : 12;
  s->sound->setVolume(MIN_VOLUME);

  // dp
  s->scene.dp_alert_rate = 0;
  s->scene.dp_alert_type = 1;
  bool show_layer = true;

  while (!do_exit) {
    if (!s->started) {
      util::sleep_for(50);
    }
    double u1 = millis_since_boot();

    ui_update(s);
    // hide layer when waze is running
    if (s->scene.dpFullScreenApp) {
      if (!show_layer && !s->started && s->status == STATUS_OFFROAD) {
        framebuffer_swap_layer(s->fb, 0);
        show_layer = true;
      } else if (show_layer && s->started) {
        framebuffer_swap_layer(s->fb, 0x00001000);
        show_layer = false;
      }
    }
    if (s->started && s->scene.dpFullScreenApp) {
      // always collapsed sidebar when vision is connect and in waze mode
      s->scene.uilayout_sidebarcollapsed = true;
    } else {
    // poll for touch events
    int touch_x = -1, touch_y = -1;
    int touched = touch_poll(&touch, &touch_x, &touch_y, 0);
    if (touched == 1) {
      if (!handle_dp_btn_touch(s, touch_x, touch_y)) {
      handle_sidebar_touch(s, touch_x, touch_y);
      handle_vision_touch(s, touch_x, touch_y);
      }
    }

    // Don't waste resources on drawing in case screen is off
    handle_display_state(s, touched == 1);
    }
    if (!s->awake) {
      continue;
    }

    // up one notch every 5 m/s
    float min = MIN_VOLUME + s->scene.controls_state.getVEgo() / 5;
    if (s->scene.dpUiVolumeBoost != 0) {
      min = min * (1 + s->scene.dpUiVolumeBoost * 0.01);
    }
    s->sound->setVolume(fmin(MAX_VOLUME, min)); // up one notch every 5 m/s

    // set brightness
    if (s->scene.dpUiBrightness == 0) {
      float clipped_brightness = fmin(512, (s->light_sensor*brightness_m) + brightness_b);
      smooth_brightness = fmin(255, clipped_brightness * 0.01 + smooth_brightness * 0.99);
      ui_set_brightness(s, (int)smooth_brightness);
    } else {
      ui_set_brightness(s, (int)(255*s->scene.dpUiBrightness*0.01));
    }
    update_offroad_layout_state(s, pm);

    // skip refresh when running waze
    if (s->scene.dpFullScreenApp && s->started) {
      continue;
    }
    ui_draw(s);
    double u2 = millis_since_boot();
    if (!s->scene.frontview && (u2-u1 > 132)) {
      // warn on sub 7fps
      LOGW("slow frame(%llu) time: %.2f", (s->sm)->frame, u2-u1);
    }
    s->fb->swap();
  }

  handle_display_state(s, true);
  delete s->sm;
  delete pm;
  return 0;
}
