#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

#include <string>
#include <sstream>
#include <fstream>
#include <mutex>
#include <thread>

#include <GLES3/gl3.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "nanovg.h"
#define NANOVG_GLES3_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"


#include "common/framebuffer.h"
#include "common/touch.h"
#include "common/utilpp.h"

#define USER_AGENT "NEOSUpdater-0.2"

#define MANIFEST_URL_EON_STAGING "https://github.com/commaai/eon-neos/raw/master/update.staging.json"
#define MANIFEST_URL_EON_LOCAL "http://192.168.5.1:8000/neosupdate/update.local.json"
#define MANIFEST_URL_EON "https://github.com/commaai/eon-neos/raw/master/update.json"
const char *manifest_url = MANIFEST_URL_EON;

#define RECOVERY_DEV "/dev/block/bootdevice/by-name/recovery"
#define RECOVERY_COMMAND "/cache/recovery/command"

#define UPDATE_DIR "/data/neoupdate"

#define BUFSIZE 128

extern const uint8_t bin_opensans_regular[] asm("_binary_opensans_regular_ttf_start");
extern const uint8_t bin_opensans_regular_end[] asm("_binary_opensans_regular_ttf_end");
extern const uint8_t bin_opensans_semibold[] asm("_binary_opensans_semibold_ttf_start");
extern const uint8_t bin_opensans_semibold_end[] asm("_binary_opensans_semibold_ttf_end");
extern const uint8_t bin_opensans_bold[] asm("_binary_opensans_bold_ttf_start");
extern const uint8_t bin_opensans_bold_end[] asm("_binary_opensans_bold_ttf_end");

namespace {

bool check_battery() {
  std::string bat_cap_s = util::read_file("/sys/class/power_supply/battery/capacity");
  int bat_cap = atoi(bat_cap_s.c_str());
  std::string current_now_s = util::read_file("/sys/class/power_supply/battery/current_now");
  int current_now = atoi(current_now_s.c_str());
  return bat_cap > 35 || (current_now < 0 && bat_cap > 10);
}

bool check_space() {
  struct statvfs stat;
  if (statvfs("/data/", &stat) != 0) {
    return false;
  }
  size_t space = stat.f_bsize * stat.f_bavail;
  return space > 2000000000ULL; // 2GB
}

static void start_settings_activity(const char* name) {
  char launch_cmd[1024];
  snprintf(launch_cmd, sizeof(launch_cmd),
           "am start -W --ez :settings:show_fragment_as_subsetting true -n 'com.android.settings/.%s'", name);
  system(launch_cmd);
}

struct Updater {
  bool do_exit = false;

  TouchState touch;

  int fb_w, fb_h;
  EGLDisplay display;
  EGLSurface surface;

  FramebufferState *fb = NULL;
  NVGcontext *vg = NULL;
  int font_regular;
  int font_semibold;
  int font_bold;

  std::thread update_thread_handle;

  std::mutex lock;

  // i hate state machines give me coroutines already
  enum UpdateState {
    CONFIRMATION,
    RUNNING,
    ERROR,
  };
  UpdateState state;

  std::string progress_text;
  std::string shell_text;

  float progress_frac;

  std::string error_text;

  // button
  int b_x, b_w, b_y, b_h;
  int balt_x;

  Updater() {
    touch_init(&touch);

    //mount system and data rw so apt can do its thing
    system("mount -o rw,remount /system");
    system("mount -o rw,remount /data");

    fb = framebuffer_init("updater", 0x00001000, false,
                          &display, &surface, &fb_w, &fb_h);
    assert(fb);

    vg = nvgCreateGLES3(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
    assert(vg);

    font_regular = nvgCreateFontMem(vg, "opensans_regular", (unsigned char*)bin_opensans_regular, (bin_opensans_regular_end - bin_opensans_regular), 0);
    assert(font_regular >= 0);

    font_semibold = nvgCreateFontMem(vg, "opensans_semibold", (unsigned char*)bin_opensans_semibold, (bin_opensans_semibold_end - bin_opensans_semibold), 0);
    assert(font_semibold >= 0);

    font_bold = nvgCreateFontMem(vg, "opensans_bold", (unsigned char*)bin_opensans_bold, (bin_opensans_bold_end - bin_opensans_bold), 0);
    assert(font_bold >= 0);

    b_w = 640;
    balt_x = 200;
    b_x = fb_w-b_w-200;
    b_y = 720;
    b_h = 220;

    state = CONFIRMATION;

  }

  void set_progress(std::string text) {
    std::lock_guard<std::mutex> guard(lock);
    progress_text = text;
  }

  void set_shell(std::string text) {
    std::lock_guard<std::mutex> guard(lock);
    shell_text = text;
  }

  void set_error(std::string text) {
    std::lock_guard<std::mutex> guard(lock);
    error_text = text;
    state = ERROR;
  }


int run_command(char const *cmd, float denominator) {

    char buf[BUFSIZE];
    FILE *fp;
    float frac = 0;

    if ((fp = popen(cmd, "r")) == NULL) {
        set_shell("Error opening pipe!\n");
        return -1;
    }

    while (fgets(buf, BUFSIZE, fp) != NULL) {
        // Do whatever you want here...
        set_shell(buf);
        frac++;
        progress_frac = frac / denominator;
    }

    if(pclose(fp))  {
        set_shell("Command not found or exited with error status\n");
        return -1;
    }

    return 0;
}

  void run_stages() {

    char const *command1 = "bash /data/openpilot/installer/scipy_installer/installerscript.sh";
    char const *command2 = "Installing scipy";


    if (!check_battery()) {
      set_error("Please plug power in to your EON and wait for charge");
      return;
    }

    if (!check_space()) {
      set_error("2GB of free space required to update");
      return;
    }

    set_progress("Updating apt");
    run_command(command1, 40);


    set_progress(command2);
    run_command("apt install -y scipy", 38);

    set_progress("Installation complete");
    shell_text = "";

    system("reboot");
  }

  void draw_ack_screen(const char *title, const char *message, const char *button, const char *altbutton) {
    nvgFillColor(vg, nvgRGBA(255,255,255,255));
    nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

    nvgFontFace(vg, "opensans_bold");
    nvgFontSize(vg, 120.0f);
    nvgTextBox(vg, 110, 220, fb_w-240, title, NULL);

    nvgFontFace(vg, "opensans_regular");
    nvgFontSize(vg, 86.0f);
    nvgTextBox(vg, 130, 380, fb_w-260, message, NULL);

    // draw button
    if (button) {
      nvgBeginPath(vg);
      nvgFillColor(vg, nvgRGBA(8, 8, 8, 255));
      nvgRoundedRect(vg, b_x, b_y, b_w, b_h, 20);
      nvgFill(vg);

      nvgFillColor(vg, nvgRGBA(255, 255, 255, 255));
      nvgFontFace(vg, "opensans_semibold");
      nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
      nvgText(vg, b_x+b_w/2, b_y+b_h/2, button, NULL);

      nvgBeginPath(vg);
      nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
      nvgStrokeWidth(vg, 5);
      nvgRoundedRect(vg, b_x, b_y, b_w, b_h, 20);
      nvgStroke(vg);
    }

    // draw button
    if (altbutton) {
      nvgBeginPath(vg);
      nvgFillColor(vg, nvgRGBA(8, 8, 8, 255));
      nvgRoundedRect(vg, balt_x, b_y, b_w, b_h, 20);
      nvgFill(vg);

      nvgFillColor(vg, nvgRGBA(255, 255, 255, 255));
      nvgFontFace(vg, "opensans_semibold");
      nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
      nvgText(vg, balt_x+b_w/2, b_y+b_h/2, altbutton, NULL);

      nvgBeginPath(vg);
      nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
      nvgStrokeWidth(vg, 5);
      nvgRoundedRect(vg, balt_x, b_y, b_w, b_h, 20);
      nvgStroke(vg);
    }
  }

  void draw_progress_screen() {
    // draw progress message
    nvgFontSize(vg, 64.0f);
    nvgFillColor(vg, nvgRGBA(255,255,255,255));
    nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    nvgFontFace(vg, "opensans_bold");
    nvgFontSize(vg, 86.0f);
    nvgTextBox(vg, 0, 380, fb_w, progress_text.c_str(), NULL);

    // draw progress bar
    {
      int progress_width = 1000;
      int progress_x = fb_w/2-progress_width/2;
      int progress_y = 520;
      int progress_height = 50;

      int powerprompt_y = 312;
      nvgFontFace(vg, "opensans_regular");
      nvgFontSize(vg, 48.0f);
      nvgText(vg, fb_w/2, 740, shell_text.c_str(), NULL);

      NVGpaint paint = nvgBoxGradient(
          vg, progress_x + 1, progress_y + 1,
          progress_width - 2, progress_height, 3, 4, nvgRGB(27, 27, 27), nvgRGB(27, 27, 27));
      nvgBeginPath(vg);
      nvgRoundedRect(vg, progress_x, progress_y, progress_width, progress_height, 12);
      nvgFillPaint(vg, paint);
      nvgFill(vg);

      float value = std::min(std::max(0.0f, progress_frac), 1.0f);
      int bar_pos = ((progress_width - 2) * value);

      paint = nvgBoxGradient(
          vg, progress_x, progress_y,
          bar_pos+1.5f, progress_height-1, 3, 4,
          nvgRGB(245, 245, 245), nvgRGB(105, 105, 105));

      nvgBeginPath(vg);
      nvgRoundedRect(
          vg, progress_x+1, progress_y+1,
          bar_pos, progress_height-2, 12);
      nvgFillPaint(vg, paint);
      nvgFill(vg);
    }
  }

  void ui_draw() {
    std::lock_guard<std::mutex> guard(lock);

    nvgBeginFrame(vg, fb_w, fb_h, 1.0f);

    switch (state) {
    case CONFIRMATION:
      draw_ack_screen("Additional software is required.",
                      "Your device will now download and install this package. This will modify NEOS. You will need to connect to WiFi to download the software. Existing data on device should not be lost.",
                      "Continue",
                      "Connect to WiFi");
      break;
    case RUNNING:
      draw_progress_screen();
      break;
    case ERROR:
      draw_ack_screen("There was an error.", ("ERROR: " + error_text + "\n\nYou will need to retry").c_str(), NULL, "exit");
      break;
    }

    nvgEndFrame(vg);
  }

  void ui_update() {
    std::lock_guard<std::mutex> guard(lock);

    switch (state) {
    case ERROR:
    case CONFIRMATION: {
      int touch_x = -1, touch_y = -1;
      int res = touch_poll(&touch, &touch_x, &touch_y, 0);
      if (res == 1 && !is_settings_active()) {
        if (touch_x >= b_x && touch_x < b_x+b_w && touch_y >= b_y && touch_y < b_y+b_h) {
          if (state == CONFIRMATION) {
            state = RUNNING;
            update_thread_handle = std::thread(&Updater::run_stages, this);
          }
        }
        if (touch_x >= balt_x && touch_x < balt_x+b_w && touch_y >= b_y && touch_y < b_y+b_h) {
          if (state == CONFIRMATION) {
            start_settings_activity("Settings$WifiSettingsActivity");
          } else if (state == ERROR) {
            do_exit = 1;
          }
        }
      }
    }
    default:
      break;
    }
  }


  void go() {
    while (!do_exit) {
      ui_update();

      glClearColor(0.08, 0.08, 0.08, 1.0);
      glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

      // background
      nvgBeginPath(vg);
      NVGpaint bg = nvgLinearGradient(vg, fb_w, 0, fb_w, fb_h,
        nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 255));
      nvgFillPaint(vg, bg);
      nvgRect(vg, 0, 0, fb_w, fb_h);
      nvgFill(vg);

      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      ui_draw();

      glDisable(GL_BLEND);

      eglSwapBuffers(display, surface);
      assert(glGetError() == GL_NO_ERROR);

      // no simple way to do 30fps vsync with surfaceflinger...
      usleep(30000);
    }

    if (update_thread_handle.joinable()) {
      update_thread_handle.join();
    }

    system("service call power 16 i32 0 i32 0 i32 1");
  }

  bool is_settings_active() {
    FILE *fp;
    char sys_output[4096];

    fp = popen("/bin/dumpsys window windows", "r");
    if (fp == NULL) {
      return false;
    }

    bool active = false;
    while (fgets(sys_output, sizeof(sys_output), fp) != NULL) {
      if (strstr(sys_output, "mCurrentFocus=null")  != NULL) {
        break;
      }

      if (strstr(sys_output, "mCurrentFocus=Window") != NULL) {
        active = true;
        break;
      }
    }

    pclose(fp);

    return active;
  }

};

}
int main(int argc, char *argv[]) {
  if (argc > 1) {
    if (strcmp(argv[1], "local") == 0) {
      manifest_url = MANIFEST_URL_EON_LOCAL;
    } else if (strcmp(argv[1], "staging") == 0) {
      manifest_url = MANIFEST_URL_EON_STAGING;
    } else {
      manifest_url = argv[1];
    }
  }
  printf("updating from %s\n", manifest_url);
  Updater updater;
  updater.go();

  return 0;
}
