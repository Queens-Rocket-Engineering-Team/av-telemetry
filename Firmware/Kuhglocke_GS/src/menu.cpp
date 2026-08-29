#include "menu.h"
#include "pinouts.h"

static constexpr uint32_t kDebounceMs = 150;

// ADC midpoints between adjacent button voltages (12-bit, 3.3V ref)
static constexpr uint16_t kAdcEnterLow  = 310;   // below = None
static constexpr uint16_t kAdcEnterHigh = 993;   // ENTER: 0.5V ≈ 620
static constexpr uint16_t kAdcBackHigh  = 1738;  // BACK:  1.1V ≈ 1365
static constexpr uint16_t kAdcDownHigh  = 2420;  // DOWN:  1.7V ≈ 2110
static constexpr uint16_t kAdcUpHigh    = 3100;  // UP:    2.2V ≈ 2730

static MenuScreen s_screen = MenuScreen::Telemetry;
static MenuButton s_lastRaw = MenuButton::None;
static uint32_t   s_lastChangeMs = 0;
static bool       s_pressed = false;  // true while button is held after debounce
static bool       s_editing = false;  // Radio Config edit mode
static uint8_t    s_editParam = 0;    // which radio param is selected (0-3)

static MenuButton readRaw() {
  uint16_t adc = analogRead(pins::kMenuBtns);
  if (adc < kAdcEnterLow)  return MenuButton::None;
  if (adc < kAdcEnterHigh) return MenuButton::Enter;
  if (adc < kAdcBackHigh)  return MenuButton::Back;
  if (adc < kAdcDownHigh)  return MenuButton::Down;
  if (adc < kAdcUpHigh)    return MenuButton::Up;
  return MenuButton::None;
}

static constexpr uint8_t kScreenCount = static_cast<uint8_t>(MenuScreen::kCount);

void menuInit() {
  s_screen = MenuScreen::Telemetry;
  s_editing = false;
  s_editParam = 0;
}

MenuScreen menuCurrentScreen() {
  return s_screen;
}

MenuButton menuReadButton() {
  return s_lastRaw;
}

// Forward declarations for radio editing helpers (implemented in radio_control)
void radioEditBegin();
void radioEditMove(int8_t dir);
void radioEditConfirm();
void radioEditCancel();
uint8_t radioEditParam();
bool radioEditActive();

void menuService(uint32_t nowMs) {
  MenuButton raw = readRaw();

  if (raw != s_lastRaw) {
    s_lastRaw = raw;
    s_lastChangeMs = nowMs;
    s_pressed = false;
    return;
  }

  if (s_pressed) return;
  if (nowMs - s_lastChangeMs < kDebounceMs) return;
  if (raw == MenuButton::None) return;

  // Debounced press event
  s_pressed = true;

  if (radioEditActive()) {
    // Inside radio parameter editing
    switch (raw) {
      case MenuButton::Up:    radioEditMove(1);     break;
      case MenuButton::Down:  radioEditMove(-1);    break;
      case MenuButton::Enter: radioEditConfirm();   break;
      case MenuButton::Back:  radioEditCancel();    break;
      default: break;
    }
    return;
  }

  switch (raw) {
    case MenuButton::Up: {
      uint8_t s = static_cast<uint8_t>(s_screen);
      s_screen = static_cast<MenuScreen>((s + kScreenCount - 1) % kScreenCount);
      break;
    }
    case MenuButton::Down: {
      uint8_t s = static_cast<uint8_t>(s_screen);
      s_screen = static_cast<MenuScreen>((s + 1) % kScreenCount);
      break;
    }
    case MenuButton::Back:
      s_screen = MenuScreen::Telemetry;
      break;
    case MenuButton::Enter:
      if (s_screen == MenuScreen::Radio) {
        radioEditBegin();
      }
      break;
    default:
      break;
  }
}
